#include "cartan/lie/se3.h"
#include "cartan/analytical/solver_2r.h"
#include "cartan/analytical/solver_3r.h"
#include "cartan/analytical/solver_6r.h"
#include "cartan/analytical/paden_kahan.h"
#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/ik_validation.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "registrations.h"
#include "detail/expected_caster.h"
#include "detail/analytical_python_helpers.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>
#include <nanobind/nanobind.h>

#include <optional>
#include <utility>
#include <sstream>
#include <algorithm>
#include <cstddef>
#include <limits>

namespace nb = nanobind;

namespace
{

using KC       = cartan::kinematic_chain<double, cartan::dynamic>;
using SE3d     = cartan::se3<double>;
using Vec3d    = Eigen::Matrix<double, 3, 1>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

/// Guard each solver lambda against NaN / non-finite target components.
/// Hard fails raise Python ValueError per the plan's input contract;
/// joint-count / geometry mismatches are soft fails that flow through the
/// solver and surface as AnalyticalResult.status.
inline void validate_target_finite(const char* fn_name, const SE3d& target)
{
    if (!target.translation().array().isFinite().all())
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
}

/// L2 distance between two joint vectors after size check. Returns +inf
/// on size mismatch so ranking falls back to a usable order rather than
/// throwing inside std::sort or std::min_element comparators.
inline double l2_distance(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    if (a.size() != b.size())
    {
        return std::numeric_limits<double>::infinity();
    }
    return (a - b).norm();
}

}

namespace cartan::python
{

void register_analytical(nb::module_& m)
{
    nb::module_ analytical = m.def_submodule(
        "analytical",
        "Closed-form analytical IK solvers and Paden-Kahan subproblems.");

    // ------------------------------------------------------------------
    // AnalyticalStatus enum (5 variants mirroring C++ analytical_failure
    // plus a Python success sentinel "ok"). The set is fixed to mirror
    // cartan::analytical_failure; the Python surface intentionally does
    // not expose any legacy "singular" / "near_singular" names.
    // ------------------------------------------------------------------
    nb::enum_<py_analytical_status>(analytical, "AnalyticalStatus",
        "Outcome of an analytical IK solve. ok signals success; the four "
        "failure variants mirror cartan::analytical_failure.")
        .value("ok",                     py_analytical_status::ok)
        .value("unreachable",            py_analytical_status::unreachable)
        .value("degenerate_geometry",    py_analytical_status::degenerate_geometry)
        .value("singular_configuration", py_analytical_status::singular_configuration)
        .value("verification_failed",    py_analytical_status::verification_failed);

    // ------------------------------------------------------------------
    // AnalyticalResult value class (def_ro on the three fields). The
    // solutions list is populated on the success path; status carries
    // the coarse outcome; error_metric is the workspace_distance from
    // the C++ analytical_error on the unreachable branch and 0.0 on
    // the success path.
    // ------------------------------------------------------------------
    nb::class_<AnalyticalResult>(analytical, "AnalyticalResult",
        "Closed-form solve outcome with always-populated fields. "
        "solutions is the list of joint vectors that survived FK back-check; "
        "status names the coarse outcome; error_metric is the workspace "
        "distance magnitude when status == unreachable, otherwise 0.0.")
        .def_ro("solutions",    &AnalyticalResult::solutions)
        .def_ro("status",       &AnalyticalResult::status)
        .def_ro("error_metric", &AnalyticalResult::error_metric)
        .def("__repr__",
            [](const AnalyticalResult& r) {
                std::ostringstream s;
                s << "AnalyticalResult(num_solutions=" << r.solutions.size()
                  << ", status=" << static_cast<int>(r.status)
                  << ", error_metric=" << r.error_metric << ")";
                return s.str();
            });

    // ------------------------------------------------------------------
    // Solver lambdas: solve_pieper_6r, solve_planar_2r, solve_3r.
    // Each constructs a fresh solver from the dynamic chain via CTAD on
    // the chain concept, calls solve(target), and unwraps the
    // expected<...> via to_analytical_result. Hard-fail target validation
    // happens before the GIL is released so the ValueError is raised on
    // the calling thread.
    // ------------------------------------------------------------------
    analytical.def("solve_pieper_6r",
        [](const KC& chain, const SE3d& target) -> AnalyticalResult {
            validate_target_finite("solve_pieper_6r", target);
            cartan::pieper_6r_solver solver(chain);
            return to_analytical_result(solver.solve(target));
        },
        "Closed-form 6R inverse kinematics for Pieper-type wrists. "
        "Returns up to 8 FK-verified branches; on a non-Pieper chain "
        "status is degenerate_geometry and solutions is empty.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::call_guard<nb::gil_scoped_release>());

    analytical.def("solve_planar_2r",
        [](const KC& chain, const SE3d& target) -> AnalyticalResult {
            validate_target_finite("solve_planar_2r", target);
            cartan::planar_2r_solver solver(chain);
            return to_analytical_result(solver.solve(target));
        },
        "Closed-form planar 2R inverse kinematics. Returns up to two "
        "FK-verified branches (elbow-up and elbow-down).",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::call_guard<nb::gil_scoped_release>());

    analytical.def("solve_3r",
        [](const KC& chain, const SE3d& target) -> AnalyticalResult {
            validate_target_finite("solve_3r", target);
            cartan::spatial_3r_solver solver(chain);
            return to_analytical_result(solver.solve(target));
        },
        "Closed-form spatial 3R position-only inverse kinematics. "
        "Returns up to four FK-verified branches.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::call_guard<nb::gil_scoped_release>());

    // ------------------------------------------------------------------
    // Paden-Kahan subproblems as free functions. Subproblem 1 returns a
    // single angle; subproblem 2 returns up to two (theta1, theta2)
    // pairs; subproblem 3 returns up to two angles. Each surfaces the
    // C++ expected<...> as either the value or std::nullopt (subproblems
    // 2 and 3) / NaN (subproblem 1) — exposing the failure variants
    // through the existing analytical_failure-to-string convention
    // would require a side-channel; the optional / NaN shape is the
    // canonical Pinocchio / KDL convention for teaching-grade subproblem
    // access.
    // ------------------------------------------------------------------
    analytical.def("paden_kahan_1",
        [](const Vec3d& omega, const Vec3d& q,
           const Vec3d& p, const Vec3d& p_prime) -> std::optional<double> {
            auto r = cartan::paden_kahan_1<double>(omega, q, p, p_prime);
            if (!r)
                return std::nullopt;
            return *r;
        },
        "Paden-Kahan subproblem 1: find theta such that exp([omega]*theta) "
        "applied at q maps p to p'. Returns None when the constraint has no "
        "solution (the two points are not equidistant from the axis).",
        nb::arg("omega").noconvert(),
        nb::arg("q").noconvert(),
        nb::arg("p").noconvert(),
        nb::arg("p_prime").noconvert(),
        nb::call_guard<nb::gil_scoped_release>());

    analytical.def("paden_kahan_2",
        [](const Vec3d& omega1, const Vec3d& omega2, const Vec3d& q,
           const Vec3d& p, const Vec3d& p_prime)
            -> std::vector<std::pair<double, double>> {
            auto r = cartan::paden_kahan_2<double>(omega1, omega2, q, p, p_prime);
            std::vector<std::pair<double, double>> out;
            if (!r)
                return out;
            out.reserve(static_cast<std::size_t>(r->count));
            for (int i = 0; i < r->count; ++i)
            {
                out.emplace_back(r->solutions[static_cast<std::size_t>(i)]);
            }
            return out;
        },
        "Paden-Kahan subproblem 2: find up to two (theta1, theta2) pairs "
        "such that exp([omega1]*theta1) * exp([omega2]*theta2) applied at q "
        "maps p to p'. Axes omega1 and omega2 must intersect at q.",
        nb::arg("omega1").noconvert(),
        nb::arg("omega2").noconvert(),
        nb::arg("q").noconvert(),
        nb::arg("p").noconvert(),
        nb::arg("p_prime").noconvert(),
        nb::call_guard<nb::gil_scoped_release>());

    analytical.def("paden_kahan_3",
        [](const Vec3d& omega, const Vec3d& q,
           const Vec3d& p, const Vec3d& p_prime, double delta)
            -> std::vector<double> {
            auto r = cartan::paden_kahan_3<double>(omega, q, p, p_prime, delta);
            std::vector<double> out;
            if (!r)
                return out;
            out.reserve(static_cast<std::size_t>(r->count));
            for (int i = 0; i < r->count; ++i)
            {
                out.push_back(r->solutions[static_cast<std::size_t>(i)]);
            }
            return out;
        },
        "Paden-Kahan subproblem 3: find up to two theta values such that "
        "||exp([omega]*theta)*p - p'|| == delta, with the rotation taken "
        "about the axis omega through q.",
        nb::arg("omega").noconvert(),
        nb::arg("q").noconvert(),
        nb::arg("p").noconvert(),
        nb::arg("p_prime").noconvert(),
        nb::arg("delta"),
        nb::call_guard<nb::gil_scoped_release>());

    // ------------------------------------------------------------------
    // Multi-solution helpers.
    //   closest_to_seed: argmin by L2 distance over result.solutions;
    //   verify_solution: FK back-check at a single tolerance mapped to
    //     both position and orientation tolerances of the C++ verifier;
    //   filter_valid_solutions: new AnalyticalResult keeping only the
    //     branches that survive verify_solution at the given tolerance;
    //   solve_all: dispatches by chain.num_joints() (6 -> pieper, 2 ->
    //     planar, 3 -> spatial-3r; else degenerate_geometry) and ranks
    //     the surviving branches by distance_to_seed when q_seed is
    //     supplied, otherwise leaves the solver's native order intact.
    // ------------------------------------------------------------------
    analytical.def("closest_to_seed",
        [](const AnalyticalResult& result,
           const nb::DRef<const VectorXd>& q_seed) -> std::optional<VectorXd> {
            if (result.solutions.empty())
                return std::nullopt;
            const VectorXd seed = q_seed;
            auto it = std::min_element(
                result.solutions.begin(), result.solutions.end(),
                [&seed](const VectorXd& a, const VectorXd& b) {
                    return l2_distance(a, seed) < l2_distance(b, seed);
                });
            return *it;
        },
        "Return the solution closest to q_seed by L2 distance. "
        "Returns None when result.solutions is empty.",
        nb::arg("result"),
        nb::arg("q_seed").noconvert(),
        nb::call_guard<nb::gil_scoped_release>());

    analytical.def("verify_solution",
        [](const KC& chain,
           const nb::DRef<const VectorXd>& q,
           const SE3d& target,
           double tolerance) -> bool {
            validate_target_finite("verify_solution", target);
            if (q.size() != chain.num_joints())
            {
                std::ostringstream s;
                s << "verify_solution: q.size() (" << q.size()
                  << ") does not match chain.num_joints() ("
                  << chain.num_joints() << ")";
                throw nb::value_error(s.str().c_str());
            }
            cartan::convergence_criteria<double> criteria{
                tolerance, tolerance, 0, 0};
            return cartan::verify_solution(chain, target, VectorXd(q), criteria);
        },
        "FK back-check: forward_kinematics(chain, q) must agree with target "
        "to within tolerance in both translation and rotation components of "
        "the body twist. The single tolerance argument is applied to both "
        "the position and orientation tolerances of the C++ verifier.",
        nb::arg("chain"),
        nb::arg("q").noconvert(),
        nb::arg("target").noconvert(),
        nb::arg("tolerance") = 1e-6,
        nb::call_guard<nb::gil_scoped_release>());

    analytical.def("filter_valid_solutions",
        [](const KC& chain,
           const AnalyticalResult& result,
           const SE3d& target,
           double tolerance) -> AnalyticalResult {
            validate_target_finite("filter_valid_solutions", target);
            cartan::convergence_criteria<double> criteria{
                tolerance, tolerance, 0, 0};
            AnalyticalResult filtered;
            filtered.status = result.status;
            filtered.error_metric = result.error_metric;
            filtered.solutions.reserve(result.solutions.size());
            for (const auto& q : result.solutions)
            {
                if (q.size() != chain.num_joints())
                    continue;
                if (cartan::verify_solution(chain, target, q, criteria))
                {
                    filtered.solutions.push_back(q);
                }
            }
            return filtered;
        },
        "Return a new AnalyticalResult keeping only the solutions that "
        "FK-verify against target within tolerance. The status and "
        "error_metric of the input are preserved.",
        nb::arg("chain"),
        nb::arg("result"),
        nb::arg("target").noconvert(),
        nb::arg("tolerance") = 1e-6,
        nb::call_guard<nb::gil_scoped_release>());

    analytical.def("solve_all",
        [](const KC& chain,
           const SE3d& target,
           std::optional<nb::DRef<const VectorXd>> q_seed,
           int /*rank*/) -> AnalyticalResult {
            validate_target_finite("solve_all", target);
            AnalyticalResult result;
            const int n = chain.num_joints();
            switch (n)
            {
                case 6:
                {
                    cartan::pieper_6r_solver solver(chain);
                    result = to_analytical_result(solver.solve(target));
                    break;
                }
                case 2:
                {
                    cartan::planar_2r_solver solver(chain);
                    result = to_analytical_result(solver.solve(target));
                    break;
                }
                case 3:
                {
                    cartan::spatial_3r_solver solver(chain);
                    result = to_analytical_result(solver.solve(target));
                    break;
                }
                default:
                    result.status = py_analytical_status::degenerate_geometry;
                    result.error_metric = 0.0;
                    return result;
            }

            if (q_seed.has_value() && !result.solutions.empty())
            {
                const VectorXd seed = q_seed.value();
                std::sort(result.solutions.begin(), result.solutions.end(),
                    [&seed](const VectorXd& a, const VectorXd& b) {
                        return l2_distance(a, seed) < l2_distance(b, seed);
                    });
            }
            return result;
        },
        "Dispatch by chain.num_joints() to the matching closed-form solver "
        "(6 -> Pieper, 2 -> planar-2R, 3 -> spatial-3R; else status is "
        "degenerate_geometry). When q_seed is supplied the surviving "
        "branches are sorted by ascending L2 distance to q_seed so that "
        "result.solutions[0] is the seed-nearest branch. The rank kwarg "
        "is reserved for future ranking strategies; only rank=0 "
        "(distance_to_seed) is implemented today.",
        nb::arg("chain"),
        nb::arg("target").noconvert(),
        nb::kw_only(),
        nb::arg("q_seed") = nb::none(),
        nb::arg("rank") = 0,
        nb::call_guard<nb::gil_scoped_release>());
}

}
