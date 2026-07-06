#include "cartan/lie/se3.h"
#include "cartan/serial/ik/solvers.h"
#include "cartan/serial/ik/ik_result.h"
#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/basic_ik_runner.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/wrapper/restart_wrapper.h"
#include "cartan/serial/chain/kinematic_chain.h"

#ifdef CARTAN_BUILD_ARGMIN
#include "cartan/serial/ik/solver/argmin_slsqp.h"
#include "cartan/serial/ik/solver/argmin_lm.h"
#include "cartan/serial/ik/solver/argmin_lbfgsb.h"
#endif

#include "registrations.h"
#include "detail/ik_python_helpers.h"
#include "detail/expected_caster.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/nanobind.h>

#include <sstream>
#include <utility>
#include <optional>
#include <string>

namespace nb = nanobind;

namespace
{

using KC               = cartan::kinematic_chain<double, cartan::dynamic>;
using SE3d             = cartan::se3<double>;
using VectorXd         = Eigen::Matrix<double, Eigen::Dynamic, 1>;
// speed_ik_runner / robust_ik_runner are policies (restart_wrapper<projected_lm>
// and restart_wrapper<builtin_lbfgsb>); wrap each in a single-policy
// basic_ik_runner to materialize the setup() + solve() runner surface.
// dual_ik_runner is already a basic_ik_runner racing the two policies.
using py_speed_runner  = cartan::basic_ik_runner<cartan::speed_ik_runner<KC>>;
using py_robust_runner = cartan::basic_ik_runner<cartan::robust_ik_runner<KC>>;
using py_dual_runner   = cartan::dual_ik_runner<KC>;

#ifdef CARTAN_BUILD_ARGMIN
// argmin-backed iterative IK runners. The LimitsPolicy per inner solver
// follows the C++ default for each solver class -- LM cannot tolerate
// post-step clamping because the trust-region step would no longer be the
// step computed (post-clamp truncation breaks the Cauchy-point / dogleg
// guarantees); SLSQP and L-BFGS-B both natively handle box constraints
// inside the inner QP / projected line search, so clamp_limits matches
// the inner policy's own bound-projection cadence. Each inner solver is
// wrapped in restart_wrapper for parity with the speed/robust runners
// above (Halton multi-start on stall / divergence / iteration_limit).
using py_argmin_slsqp_inner =
    cartan::restart_wrapper<KC,
        cartan::argmin_slsqp<KC, cartan::clamp_limits>,
        cartan::clamp_limits>;
using py_argmin_lm_inner =
    cartan::restart_wrapper<KC,
        cartan::argmin_lm<KC, cartan::no_limits>,
        cartan::no_limits>;
using py_argmin_lbfgsb_inner =
    cartan::restart_wrapper<KC,
        cartan::argmin_lbfgsb<KC, cartan::clamp_limits>,
        cartan::clamp_limits>;

using py_argmin_slsqp_runner  = cartan::basic_ik_runner<py_argmin_slsqp_inner>;
using py_argmin_lm_runner     = cartan::basic_ik_runner<py_argmin_lm_inner>;
using py_argmin_lbfgsb_runner = cartan::basic_ik_runner<py_argmin_lbfgsb_inner>;
#endif

/// Validate target finiteness + q_seed.size() == chain.num_joints().
/// Raises Python ValueError on failure (per the iterative IK hard-fail contract).
inline void validate_ik_inputs(const char* fn_name,
                               const KC& chain,
                               const SE3d& target,
                               const nb::DRef<const VectorXd>& q_seed)
{
    if (target.translation().array().isNaN().any() ||
        !target.translation().array().isFinite().all())
    {
        std::ostringstream s;
        s << fn_name << ": target contains NaN or non-finite translation";
        throw nb::value_error(s.str().c_str());
    }
    if (!target.rotation().matrix().array().isFinite().all())
    {
        std::ostringstream s;
        s << fn_name << ": target contains NaN or non-finite rotation";
        throw nb::value_error(s.str().c_str());
    }
    if (q_seed.size() != chain.num_joints())
    {
        std::ostringstream s;
        s << fn_name << ": q_seed.size() (" << q_seed.size()
          << ") does not match chain.num_joints() (" << chain.num_joints() << ")";
        throw nb::value_error(s.str().c_str());
    }
}

template <typename Runner>
inline cartan::python::IkResult run_ik(const KC& chain,
                                       const SE3d& target,
                                       const nb::DRef<const VectorXd>& q_seed,
                                       const cartan::python::IkConfig& cfg)
{
    cartan::convergence_criteria<double> criteria{
        cfg.position_tol,
        cfg.orientation_tol,
        cfg.max_iterations_per_attempt,
        cfg.max_total_work_units
    };
    cartan::solver_options<double> opts{
        cfg.objective,
        cfg.max_total_iterations,
        cfg.halton_seed
    };

    Runner runner;
    runner.setup(chain, target, VectorXd(q_seed), criteria, opts);
    auto r = runner.solve();

    if (!r)
    {
        return cartan::python::to_ik_result_from_error(
            std::move(r).error(), runner.iterations());
    }
    return cartan::python::to_ik_result(std::move(*r));
}

}

namespace cartan::python
{

void register_ik(nb::module_& m)
{
    // ------------------------------------------------------------------
    // Enums (mirror C++ ik_status.h)
    // ------------------------------------------------------------------

    nb::enum_<cartan::ik_objective>(m, "IkObjective",
        "Secondary optimization objective for multi-policy IK racing.")
        .value("speed",              cartan::ik_objective::speed)
        .value("min_distance",       cartan::ik_objective::min_distance)
        .value("max_manipulability", cartan::ik_objective::max_manipulability)
        .value("max_isotropy",       cartan::ik_objective::max_isotropy);

    nb::enum_<cartan::ik_termination_reason>(m, "IkTerminationReason",
        "Fine-grained terminator reported by individual IK policies. "
        "Distinguishes the inner solver's terminator (solver_*) from the "
        "runner-level terminator (iteration_limit, stall_detected, ...).")
        .value("unknown",                      cartan::ik_termination_reason::unknown)
        .value("converged",                    cartan::ik_termination_reason::converged)
        .value("iteration_limit",              cartan::ik_termination_reason::iteration_limit)
        .value("stall_detected",               cartan::ik_termination_reason::stall_detected)
        .value("divergence_detected",          cartan::ik_termination_reason::divergence_detected)
        .value("joint_limit_hit",              cartan::ik_termination_reason::joint_limit_hit)
        .value("solver_converged_pose_missed", cartan::ik_termination_reason::solver_converged_pose_missed)
        .value("solver_ftol_reached",          cartan::ik_termination_reason::solver_ftol_reached)
        .value("solver_xtol_reached",          cartan::ik_termination_reason::solver_xtol_reached)
        .value("solver_objective_stalled",     cartan::ik_termination_reason::solver_objective_stalled)
        .value("solver_roundoff_limited",      cartan::ik_termination_reason::solver_roundoff_limited)
        .value("solver_stalled",               cartan::ik_termination_reason::solver_stalled)
        .value("solver_aborted",               cartan::ik_termination_reason::solver_aborted)
        .value("solver_budget_exhausted",      cartan::ik_termination_reason::solver_budget_exhausted)
        .value("solver_max_iterations",        cartan::ik_termination_reason::solver_max_iterations)
        .value("solver_diverged",              cartan::ik_termination_reason::solver_diverged);

    nb::enum_<cartan::ik_failure>(m, "IkFailure",
        "Coarse-grained failure category reported by an IK solve when it does "
        "not converge. Mirrors C++ ik_failure for typed dispatch on the "
        "Python side.")
        .value("unreachable",           cartan::ik_failure::unreachable)
        .value("diverged",              cartan::ik_failure::diverged)
        .value("stalled",               cartan::ik_failure::stalled)
        .value("iteration_limit",       cartan::ik_failure::iteration_limit)
        .value("joint_limit_violation", cartan::ik_failure::joint_limit_violation)
        .value("aborted",               cartan::ik_failure::aborted);

    // ------------------------------------------------------------------
    // IkConfig (kw-only ctor; def_rw on each field)
    // ------------------------------------------------------------------

    nb::class_<IkConfig>(m, "IkConfig",
        "Iterative IK solver configuration (kwargs-only constructor). "
        "Defaults mirror cartan::convergence_criteria + cartan::solver_options.")
        .def("__init__",
            [](IkConfig* self,
               int max_iterations_per_attempt,
               int max_total_work_units,
               double position_tol,
               double orientation_tol,
               int max_total_iterations,
               cartan::ik_objective objective,
               unsigned int halton_seed)
            {
                new (self) IkConfig{
                    max_iterations_per_attempt,
                    max_total_work_units,
                    position_tol,
                    orientation_tol,
                    max_total_iterations,
                    objective,
                    halton_seed
                };
            },
            nb::kw_only(),
            nb::arg("max_iterations_per_attempt") = 100,
            nb::arg("max_total_work_units")       = 200,
            nb::arg("position_tol")               = 1e-6,
            nb::arg("orientation_tol")            = 1e-6,
            nb::arg("max_total_iterations")       = 500,
            nb::arg("objective")                  = cartan::ik_objective::speed,
            nb::arg("halton_seed")                = 42u)
        .def_rw("max_iterations_per_attempt", &IkConfig::max_iterations_per_attempt)
        .def_rw("max_total_work_units",       &IkConfig::max_total_work_units)
        .def_rw("position_tol",               &IkConfig::position_tol)
        .def_rw("orientation_tol",            &IkConfig::orientation_tol)
        .def_rw("max_total_iterations",       &IkConfig::max_total_iterations)
        .def_rw("objective",                  &IkConfig::objective)
        .def_rw("halton_seed",                &IkConfig::halton_seed)
        .def("__repr__",
            [](const IkConfig& c) {
                std::ostringstream s;
                s << "IkConfig(max_iterations_per_attempt=" << c.max_iterations_per_attempt
                  << ", max_total_work_units=" << c.max_total_work_units
                  << ", position_tol=" << c.position_tol
                  << ", orientation_tol=" << c.orientation_tol
                  << ", max_total_iterations=" << c.max_total_iterations
                  << ", halton_seed=" << c.halton_seed << ")";
                return s.str();
            });

    // ------------------------------------------------------------------
    // IkResult (def_ro on each field; always-returned shape)
    // ------------------------------------------------------------------

    nb::class_<IkResult>(m, "IkResult",
        "Iterative IK solve outcome with always-populated diagnostic fields. "
        "The .converged flag is the success signal; on .converged == True the "
        "q field holds the converged joint vector, error_norm the final task "
        "error magnitude, iterations the total work units charged, and "
        "termination_reason carries the policy's fine-grained terminator. "
        "On .converged == False the q field holds the best-seen position, "
        "failure_reason names the coarse category (mirror of cartan::ik_failure), "
        "and condition_number / near_singular reflect the failure-state "
        "Jacobian. condition_number is 0.0 on the success path; cartan does "
        "not currently compute it on convergence to keep the hot path "
        "Jacobian-SVD free.")
        .def_ro("q",                  &IkResult::q)
        .def_ro("converged",          &IkResult::converged)
        .def_ro("iterations",         &IkResult::iterations)
        .def_ro("error_norm",         &IkResult::error_norm)
        .def_ro("failure_reason",     &IkResult::failure_reason)
        .def_ro("solver_index",       &IkResult::solver_index)
        .def_ro("termination_reason", &IkResult::termination_reason)
        .def_ro("near_singular",      &IkResult::near_singular)
        .def_ro("condition_number",   &IkResult::condition_number)
        .def("__repr__",
            [](const IkResult& r) {
                std::ostringstream s;
                s << "IkResult(converged=" << (r.converged ? "True" : "False")
                  << ", iterations=" << r.iterations
                  << ", error_norm=" << r.error_norm
                  << ", solver_index=" << r.solver_index
                  << ", failure_reason='" << r.failure_reason << "'"
                  << ", near_singular=" << (r.near_singular ? "True" : "False")
                  << ", condition_number=" << r.condition_number << ")";
                return s.str();
            });

    // ------------------------------------------------------------------
    // Free-function solvers: solve_ik (dual race), solve_ik_speed,
    // solve_ik_robust. Each constructs a fresh runner per call to honor the
    // basic_ik_runner thread-safety contract; GIL is released around the
    // C++ solve() body so the calls scale across a ThreadPoolExecutor.
    // ------------------------------------------------------------------

    m.def("solve_ik",
        [](const KC& chain,
           const SE3d& target,
           const nb::DRef<const VectorXd>& q_seed,
           std::optional<IkConfig> config) -> IkResult
        {
            validate_ik_inputs("solve_ik", chain, target, q_seed);
            const IkConfig cfg = config.value_or(IkConfig{});
            return run_ik<py_dual_runner>(chain, target, q_seed, cfg);
        },
        "Default iterative IK: races a projected-LM speed policy against an "
        "L-BFGS-B robust policy via dual_ik_runner and returns the winning "
        "IkResult. Hard fails (NaN target, joint-count mismatch) raise "
        "ValueError; soft fails populate the IkResult with .converged=False.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::arg("q_seed").noconvert(),
        nb::arg("config") = nb::none(),
        nb::call_guard<nb::gil_scoped_release>());

    m.def("solve_ik_speed",
        [](const KC& chain,
           const SE3d& target,
           const nb::DRef<const VectorXd>& q_seed,
           std::optional<IkConfig> config) -> IkResult
        {
            validate_ik_inputs("solve_ik_speed", chain, target, q_seed);
            const IkConfig cfg = config.value_or(IkConfig{});
            return run_ik<py_speed_runner>(chain, target, q_seed, cfg);
        },
        "Speed-optimized iterative IK: restart-wrapped projected_lm policy "
        "(fast per-iteration, Halton-multi-start). Same input/output contract "
        "as solve_ik.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::arg("q_seed").noconvert(),
        nb::arg("config") = nb::none(),
        nb::call_guard<nb::gil_scoped_release>());

    m.def("solve_ik_robust",
        [](const KC& chain,
           const SE3d& target,
           const nb::DRef<const VectorXd>& q_seed,
           std::optional<IkConfig> config) -> IkResult
        {
            validate_ik_inputs("solve_ik_robust", chain, target, q_seed);
            const IkConfig cfg = config.value_or(IkConfig{});
            return run_ik<py_robust_runner>(chain, target, q_seed, cfg);
        },
        "Convergence-optimized iterative IK: restart-wrapped L-BFGS-B policy "
        "(robust convergence, Halton-multi-start). Same input/output contract "
        "as solve_ik.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::arg("q_seed").noconvert(),
        nb::arg("config") = nb::none(),
        nb::call_guard<nb::gil_scoped_release>());

#ifdef CARTAN_BUILD_ARGMIN
    // ------------------------------------------------------------------
    // argmin-backed iterative IK free functions. Only compiled when the
    // wheel is built with CARTAN_BUILD_ARGMIN=ON; cartan.has_argmin
    // exposes the build configuration as a module-level bool. Each
    // solver shares the IkResult / IkConfig contract above; per-solver
    // LimitsPolicy matches the C++ default for the inner solver class.
    // ------------------------------------------------------------------

    m.def("solve_ik_argmin_slsqp",
        [](const KC& chain,
           const SE3d& target,
           const nb::DRef<const VectorXd>& q_seed,
           std::optional<IkConfig> config) -> IkResult
        {
            validate_ik_inputs("solve_ik_argmin_slsqp", chain, target, q_seed);
            const IkConfig cfg = config.value_or(IkConfig{});
            return run_ik<py_argmin_slsqp_runner>(chain, target, q_seed, cfg);
        },
        "argmin-backed SLSQP iterative IK: restart-wrapped argmin_slsqp "
        "policy with clamp_limits (native box-constraint handling in the "
        "inner SQP QP). Same input/output contract as solve_ik. Only "
        "available when the wheel is built with CARTAN_BUILD_ARGMIN=ON; "
        "check cartan.has_argmin before calling.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::arg("q_seed").noconvert(),
        nb::arg("config") = nb::none(),
        nb::call_guard<nb::gil_scoped_release>());

    m.def("solve_ik_argmin_lm",
        [](const KC& chain,
           const SE3d& target,
           const nb::DRef<const VectorXd>& q_seed,
           std::optional<IkConfig> config) -> IkResult
        {
            validate_ik_inputs("solve_ik_argmin_lm", chain, target, q_seed);
            const IkConfig cfg = config.value_or(IkConfig{});
            return run_ik<py_argmin_lm_runner>(chain, target, q_seed, cfg);
        },
        "argmin-backed Levenberg-Marquardt iterative IK: restart-wrapped "
        "argmin_lm policy with no_limits (LM trust-region semantics break "
        "when joint values are clamped after a step, so the limits policy "
        "is left as no_limits to match the C++ default). Same "
        "input/output contract as solve_ik. Only available when the wheel "
        "is built with CARTAN_BUILD_ARGMIN=ON; check cartan.has_argmin "
        "before calling.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::arg("q_seed").noconvert(),
        nb::arg("config") = nb::none(),
        nb::call_guard<nb::gil_scoped_release>());

    m.def("solve_ik_argmin_lbfgsb",
        [](const KC& chain,
           const SE3d& target,
           const nb::DRef<const VectorXd>& q_seed,
           std::optional<IkConfig> config) -> IkResult
        {
            validate_ik_inputs("solve_ik_argmin_lbfgsb", chain, target, q_seed);
            const IkConfig cfg = config.value_or(IkConfig{});
            return run_ik<py_argmin_lbfgsb_runner>(chain, target, q_seed, cfg);
        },
        "argmin-backed L-BFGS-B iterative IK: restart-wrapped "
        "argmin_lbfgsb policy with clamp_limits (native box-constraint "
        "handling in the inner projected line search). Same input/output "
        "contract as solve_ik. Only available when the wheel is built "
        "with CARTAN_BUILD_ARGMIN=ON; check cartan.has_argmin before "
        "calling.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::arg("q_seed").noconvert(),
        nb::arg("config") = nb::none(),
        nb::call_guard<nb::gil_scoped_release>());
#endif
}

}
