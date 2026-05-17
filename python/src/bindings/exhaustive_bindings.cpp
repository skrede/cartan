#include "cartan/lie/se3.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/ik/solver/exhaustive_ik_runner.h"
#include "cartan/serial/ik/solver/projected_lm.h"
#include "cartan/serial/ik/solver/lbfgsb.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/ik_status.h"

#include "registrations.h"
#include "detail/ik_python_helpers.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>
#include <nanobind/nanobind.h>

#include <variant>
#include <utility>
#include <optional>
#include <sstream>
#include <vector>

namespace nb = nanobind;

namespace
{

using KC       = cartan::kinematic_chain<double, cartan::dynamic>;
using SE3d     = cartan::se3<double>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

/// Inner solve policies are RAW (NOT restart_wrapper-wrapped). The exhaustive
/// runner is itself the restart loop: it constructs a fresh `Policy{}` per
/// iteration at exhaustive_ik_runner.h:77. Wrapping the policy in
/// restart_wrapper here would double the restart loop and silently degrade
/// the multi-start budget. See CONTEXT.md correction C-08.
using ExSpeedInner   = cartan::ik::projected_lm<KC, cartan::no_limits>;
using ExRobustInner  = cartan::ik::builtin_lbfgsb<KC, cartan::no_limits>;
using ExSpeedRunner  = cartan::exhaustive_ik_runner<KC, ExSpeedInner>;
using ExRobustRunner = cartan::exhaustive_ik_runner<KC, ExRobustInner>;
using ExVariant      = std::variant<ExSpeedRunner, ExRobustRunner>;

/// Two-valued policy selector mapped to the std::variant alternative at ctor
/// time. Kept TU-local: Python sees the bound enum surface, the C++ type is
/// only consumed by the PyExhaustiveRunner ctor lambda.
enum class IkPolicy : int
{
    speed,
    robust
};

/// Binding-internal value class wrapping the chain, the IK config snapshot,
/// and the std::variant<SpeedRunner, RobustRunner> alternative chosen at
/// ctor time. `.solve()` dispatches via std::visit.
struct PyExhaustiveRunner
{
    KC chain;
    cartan::python::IkConfig config;
    ExVariant runner;

    PyExhaustiveRunner(KC c,
                       IkPolicy policy,
                       std::optional<cartan::python::IkConfig> cfg)
        : chain(std::move(c))
        , config(cfg.value_or(cartan::python::IkConfig{}))
        , runner(policy == IkPolicy::speed
                    ? ExVariant{ExSpeedRunner{}}
                    : ExVariant{ExRobustRunner{}})
    {
    }
};

}

namespace cartan::python
{

void register_exhaustive(nb::module_& m)
{
    // ------------------------------------------------------------------
    // Enum: IkPolicy (binding-internal two-valued selector for the
    // ExhaustiveIKRunner's inner solve policy).
    // ------------------------------------------------------------------

    nb::enum_<IkPolicy>(m, "IkPolicy",
        "Inner solve policy for ExhaustiveIKRunner. `speed` selects the "
        "projected Levenberg-Marquardt policy (fast per-iteration); "
        "`robust` selects the L-BFGS-B policy (convergence-optimized).")
        .value("speed",  IkPolicy::speed)
        .value("robust", IkPolicy::robust);

    // ------------------------------------------------------------------
    // Enum: RankingStrategy (mirrors cartan::ranking_strategy from
    // exhaustive_ik_runner.h). Selects how multi-branch solutions are
    // sorted before returning to Python.
    // ------------------------------------------------------------------

    nb::enum_<cartan::ranking_strategy>(m, "RankingStrategy",
        "Ranking strategy for ExhaustiveIKRunner solution branches. "
        "distance_to_seed sorts by ||q - q_seed||_2 ascending; min_error "
        "sorts by IkResult.error_norm ascending; mid_range sorts by L2 "
        "distance to the joint-limit midpoint of each joint.")
        .value("distance_to_seed", cartan::ranking_strategy::distance_to_seed)
        .value("min_error",        cartan::ranking_strategy::min_error)
        .value("mid_range",        cartan::ranking_strategy::mid_range);

    // ------------------------------------------------------------------
    // ExhaustiveIKRunner: Halton-seeded multi-start IK runner returning a
    // ranked list of FK-verified branches. Wraps std::variant<SpeedRunner,
    // RobustRunner>; ctor picks the alternative from the policy kwarg.
    // ------------------------------------------------------------------

    nb::class_<PyExhaustiveRunner>(m, "ExhaustiveIKRunner",
        "Halton-seeded multi-start iterative IK runner returning a ranked "
        "list of FK-verified branches. Internally wraps "
        "std::variant<exhaustive_ik_runner<KC, projected_lm<KC, no_limits>>, "
        "exhaustive_ik_runner<KC, builtin_lbfgsb<KC, no_limits>>>; the ctor "
        "picks the variant alternative from the policy kwarg. Inner policies "
        "are raw (the runner itself is the restart loop).")
        .def(nb::init<KC, IkPolicy, std::optional<IkConfig>>(),
             nb::arg("chain"),
             nb::kw_only(),
             nb::arg("policy") = IkPolicy::speed,
             nb::arg("config") = nb::none(),
             "Construct an ExhaustiveIKRunner wrapping the given chain. "
             "`policy=cartan.IkPolicy.speed` (default) selects projected LM; "
             "`policy=cartan.IkPolicy.robust` selects L-BFGS-B. The optional "
             "`config` snapshot is consumed by each .solve() call.")
        .def("solve",
            [](PyExhaustiveRunner& self,
               const SE3d& target,
               std::optional<nb::DRef<const VectorXd>> q_seed,
               int max_restarts,
               double dedup_tolerance,
               cartan::ranking_strategy ranking) -> std::vector<IkResult>
            {
                // Hard-fail guards: NaN / non-finite target components.
                if (!target.translation().array().isFinite().all())
                {
                    std::ostringstream s;
                    s << "ExhaustiveIKRunner.solve: target contains NaN or "
                         "non-finite translation";
                    throw nb::value_error(s.str().c_str());
                }
                if (!target.rotation().matrix().array().isFinite().all())
                {
                    std::ostringstream s;
                    s << "ExhaustiveIKRunner.solve: target contains NaN or "
                         "non-finite rotation";
                    throw nb::value_error(s.str().c_str());
                }

                VectorXd seed = q_seed
                    ? VectorXd(*q_seed)
                    : VectorXd::Zero(self.chain.num_joints());
                if (seed.size() != self.chain.num_joints())
                {
                    std::ostringstream s;
                    s << "ExhaustiveIKRunner.solve: q_seed.size() ("
                      << seed.size() << ") does not match chain.num_joints() ("
                      << self.chain.num_joints() << ")";
                    throw nb::value_error(s.str().c_str());
                }

                cartan::convergence_criteria<double> criteria{
                    self.config.position_tol,
                    self.config.orientation_tol,
                    self.config.max_iterations_per_attempt,
                    self.config.max_total_work_units
                };
                cartan::exhaustive_options<double> opts{
                    max_restarts,
                    dedup_tolerance,
                    ranking
                };

                return std::visit(
                    [&](auto& r) -> std::vector<IkResult>
                    {
                        auto res = r.solve(self.chain, target, seed,
                                           criteria, opts);
                        std::vector<IkResult> out;
                        out.reserve(res.solutions.size());
                        for (auto& s : res.solutions)
                            out.push_back(to_ik_result(std::move(s)));
                        return out;
                    },
                    self.runner);
            },
            nb::arg("target").noconvert(),
            nb::kw_only(),
            nb::arg("q_seed")          = nb::none(),
            nb::arg("max_restarts")    = 100,
            nb::arg("dedup_tolerance") = 1e-3,
            nb::arg("ranking")         = cartan::ranking_strategy::distance_to_seed,
            nb::call_guard<nb::gil_scoped_release>(),
            "Run the exhaustive multi-start IK solver against `target` "
            "starting from `q_seed` (or the zero vector when omitted). "
            "Returns a list of FK-verified cartan.IkResult branches sorted "
            "by `ranking`; the list is empty when no branches converge. "
            "Hard fails (NaN/non-finite target, q_seed.size() mismatch) "
            "raise ValueError on the calling thread before the GIL is "
            "released.");
}

}
