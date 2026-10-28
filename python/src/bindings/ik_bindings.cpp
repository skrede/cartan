#include "registrations.h"

#include "detail/format_double.h"
#include "detail/ik_python_helpers.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/ik/solvers.h"
#include "cartan/serial/ik/ik_result.h"
#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/basic_ik_runner.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/wrapper/restart_wrapper.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/nanobind.h>

#include <string>
#include <utility>
#include <optional>

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

using cartan::python::format_double;

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
        throw nb::value_error((std::string(fn_name) + ": target contains NaN or non-finite translation").c_str());
    }
    if (!target.rotation().matrix().array().isFinite().all())
    {
        throw nb::value_error((std::string(fn_name) + ": target contains NaN or non-finite rotation").c_str());
    }
    if (q_seed.size() != chain.num_joints())
    {
        throw nb::value_error((std::string(fn_name) + ": q_seed.size() (" + std::to_string(q_seed.size())
                               + ") does not match chain.num_joints() (" + std::to_string(chain.num_joints()) + ")").c_str());
    }
    if (!q_seed.allFinite())
    {
        throw nb::value_error((std::string(fn_name) + ": q_seed contains a NaN or non-finite component").c_str());
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
        cfg.halton_seed,
        cfg.characteristic_length
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
        .value("min_error_norm",     cartan::ik_objective::min_error_norm)
        .value("min_joint_distance", cartan::ik_objective::min_joint_distance)
        .value("max_manipulability", cartan::ik_objective::max_manipulability)
        .value("max_isotropy",       cartan::ik_objective::max_isotropy);

    nb::enum_<cartan::ik_failure>(m, "IkFailure",
        "Coarse-grained failure category reported by an IK solve when it does "
        "not converge. Mirrors C++ ik_failure for typed dispatch on the "
        "Python side.")
        .value("diverged",              cartan::ik_failure::diverged)
        .value("stalled",               cartan::ik_failure::stalled)
        .value("iteration_limit",       cartan::ik_failure::iteration_limit)
        .value("joint_limit_violation", cartan::ik_failure::joint_limit_violation)
        .value("aborted",               cartan::ik_failure::aborted)
        .value("not_initialized",       cartan::ik_failure::not_initialized)
        .value("dimension_mismatch",    cartan::ik_failure::dimension_mismatch)
        .value("non_finite_input",      cartan::ik_failure::non_finite_input)
        .value("unsupported_configuration",
            cartan::ik_failure::unsupported_configuration);

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
               cartan::ik_objective objective,
               unsigned int halton_seed,
               double characteristic_length)
            {
                new (self) IkConfig{
                    max_iterations_per_attempt,
                    max_total_work_units,
                    position_tol,
                    orientation_tol,
                    objective,
                    halton_seed,
                    characteristic_length
                };
            },
            nb::kw_only(),
            nb::arg("max_iterations_per_attempt") = 100,
            nb::arg("max_total_work_units")       = 200,
            nb::arg("position_tol")               = 1e-6,
            nb::arg("orientation_tol")            = 1e-6,
            nb::arg("objective")                  = cartan::ik_objective::speed,
            nb::arg("halton_seed")                = 42u,
            nb::arg("characteristic_length")      = 1.0)
        .def_rw("max_iterations_per_attempt", &IkConfig::max_iterations_per_attempt)
        .def_rw("max_total_work_units",       &IkConfig::max_total_work_units)
        .def_rw("position_tol",               &IkConfig::position_tol)
        .def_rw("orientation_tol",            &IkConfig::orientation_tol)
        .def_rw("objective",                  &IkConfig::objective)
        .def_rw("halton_seed",                &IkConfig::halton_seed)
        .def_rw("characteristic_length",      &IkConfig::characteristic_length)
        .def("__repr__",
            [](const IkConfig& c) {
                return "IkConfig(max_iterations_per_attempt=" + std::to_string(c.max_iterations_per_attempt)
                     + ", max_total_work_units=" + std::to_string(c.max_total_work_units)
                     + ", position_tol=" + format_double(c.position_tol)
                     + ", orientation_tol=" + format_double(c.orientation_tol)
                     + ", halton_seed=" + std::to_string(c.halton_seed) + ")";
            });

    // ------------------------------------------------------------------
    // IkResult (def_ro on each field; always-returned shape)
    // ------------------------------------------------------------------

    nb::class_<IkResult>(m, "IkResult",
        "Iterative IK solve outcome with always-populated diagnostic fields. "
        "The .converged flag is the success signal; on .converged == True the "
        "q field holds the converged joint vector, error_norm the final task "
        "error magnitude, and iterations the total work units charged. "
        "On .converged == False the q field holds the best-seen position and "
        "failure_reason names the coarse category (mirror of cartan::ik_failure). "
        "selection_metric is the value the winning candidate was ranked on under "
        "selection_objective, and is None where the objective ranks nothing. "
        "For the conditioning of the Jacobian at any "
        "configuration, including a failed solve's .q, use singular_values, "
        "condition_number, manipulability, isotropy or is_near_singular.")
        .def_ro("q",                  &IkResult::q)
        .def_ro("converged",          &IkResult::converged)
        .def_ro("iterations",         &IkResult::iterations)
        .def_ro("error_norm",         &IkResult::error_norm)
        .def_ro("failure_reason",     &IkResult::failure_reason)
        .def_ro("solver_index",       &IkResult::solver_index)
        .def_ro("selection_metric",   &IkResult::selection_metric)
        .def_ro("selection_objective", &IkResult::selection_objective)
        .def("__repr__",
            [](const IkResult& r) {
                return std::string("IkResult(converged=") + (r.converged ? "True" : "False")
                     + ", iterations=" + std::to_string(r.iterations)
                     + ", error_norm=" + format_double(r.error_norm)
                     + ", solver_index=" + std::to_string(r.solver_index)
                     + ", failure_reason='" + r.failure_reason + "')";
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
}

}
