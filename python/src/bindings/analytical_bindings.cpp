#include "cartan/analytical/solver_2r.h"
#include "cartan/analytical/solver_3r.h"
#include "cartan/analytical/solver_6r.h"
#include "cartan/analytical/solver_opw.h"
#include "cartan/analytical/paden_kahan.h"
#include "cartan/analytical/range_status.h"

#include "cartan/analytical/detail/angle_unwrap.h"

#include "cartan/lie/se3.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/ik_validation.h"

#include "cartan/serial/ik/detail/limit_enforcement.h"

#include "registrations.h"

#include "detail/expected_caster.h"
#include "detail/analytical_python_helpers.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>
#include <nanobind/nanobind.h>

#include <array>
#include <limits>
#include <cstddef>
#include <utility>
#include <optional>
#include <sstream>
#include <algorithm>

namespace nb = nanobind;

namespace {

using KC             = cartan::kinematic_chain<double, cartan::dynamic>;
using SE3d           = cartan::se3<double>;
using Vec3d          = Eigen::Matrix<double, 3, 1>;
using VectorXd       = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using OPWParametersd = cartan::opw_parameters<double>;
using cartan::python::AnalyticalResult;
using cartan::python::UnwrappedResult;
using cartan::python::py_analytical_status;
using cartan::python::to_analytical_error_result;
using cartan::python::to_analytical_result;

/// Guard each solver lambda against NaN / non-finite target components.
/// Hard fails raise Python ValueError per the input contract;
/// joint-count / geometry mismatches are soft fails that flow through the
/// solver and surface as AnalyticalResult.status.
inline void validate_target_finite(const char *fn_name, const SE3d &target)
{
    if(!target.translation().array().isFinite().all())
    {
        std::ostringstream s;
        s << fn_name << ": target contains NaN or non-finite translation";
        throw nb::value_error(s.str().c_str());
    }
    if(!target.rotation().matrix().array().isFinite().all())
    {
        std::ostringstream s;
        s << fn_name << ": target contains NaN or non-finite rotation";
        throw nb::value_error(s.str().c_str());
    }
}

/// L2 distance between two joint vectors after size check. Returns +inf
/// on size mismatch so ranking falls back to a usable order rather than
/// throwing inside std::sort or std::min_element comparators.
inline double l2_distance(const Eigen::VectorXd &a, const Eigen::VectorXd &b)
{
    if(a.size() != b.size())
    {
        return std::numeric_limits<double>::infinity();
    }
    return (a - b).norm();
}

inline std::array<double, 6> to_offset_array(const std::vector<double> &values)
{
    if(values.size() != 6)
    {
        throw nb::value_error("OPWParameters: offsets must have length 6");
    }
    std::array<double, 6> out{};
    std::copy(values.begin(), values.end(), out.begin());
    return out;
}

inline std::array<signed char, 6> to_sign_array(const std::vector<int> &values)
{
    if(values.size() != 6)
    {
        throw nb::value_error("OPWParameters: sign_corrections must have length 6");
    }
    std::array<signed char, 6> out{};
    for(std::size_t i = 0; i < values.size(); ++i)
    {
        if(values[i] != -1 && values[i] != 1)
        {
            throw nb::value_error("OPWParameters: sign_corrections entries must be -1 or +1");
        }
        out[i] = static_cast<signed char>(values[i]);
    }
    return out;
}

inline OPWParametersd make_opw_parameters(double a1, double a2, double b, double c1, double c2, double c3, double c4, const std::vector<double> &offsets,
                                          const std::vector<int> &sign_corrections)
{
    OPWParametersd params{};
    params.a1               = a1;
    params.a2               = a2;
    params.b                = b;
    params.c1               = c1;
    params.c2               = c2;
    params.c3               = c3;
    params.c4               = c4;
    params.offsets          = to_offset_array(offsets);
    params.sign_corrections = to_sign_array(sign_corrections);
    return params;
}

inline std::vector<double> opw_offsets(const OPWParametersd &params)
{
    return {params.offsets.begin(), params.offsets.end()};
}

inline void set_opw_offsets(OPWParametersd &params, const std::vector<double> &offsets)
{
    params.offsets = to_offset_array(offsets);
}

inline std::vector<int> opw_sign_corrections(const OPWParametersd &params)
{
    std::vector<int> out;
    out.reserve(params.sign_corrections.size());
    for(signed char value : params.sign_corrections)
    {
        out.push_back(static_cast<int>(value));
    }
    return out;
}

inline void set_opw_sign_corrections(OPWParametersd &params, const std::vector<int> &sign_corrections)
{
    params.sign_corrections = to_sign_array(sign_corrections);
}

inline VectorXd validated_reference(const char *fn_name, const KC &chain, const std::optional<nb::DRef<const VectorXd>> &q_seed)
{
    const int n        = chain.num_joints();
    VectorXd reference = VectorXd::Zero(n);
    if(!q_seed.has_value())
    {
        return reference;
    }
    if(q_seed->size() != n)
    {
        std::ostringstream s;
        s << fn_name << ": q_seed.size() (" << q_seed->size() << ") does not match chain.num_joints() (" << n << ")";
        throw nb::value_error(s.str().c_str());
    }
    reference = q_seed.value();
    return reference;
}

inline VectorXd unwrap_solution(const KC &chain, const VectorXd &q, const VectorXd &reference)
{
    VectorXd out       = q;
    const auto &limits = chain.limits();
    const double tol   = cartan::detail::default_feasibility_tol<double>();
    for(int i = 0; i < chain.num_joints(); ++i)
    {
        if(!cartan::detail::is_zero_pitch_revolute(chain.axis(i)))
        {
            continue;
        }
        const auto idx = static_cast<std::size_t>(i);
        out(i)         = cartan::detail::unwrap_to_range_nearest(q(i), limits[idx].position_min, limits[idx].position_max, reference(i), tol);
    }
    return out;
}

inline UnwrappedResult unwrap_analytical_result(const char *fn_name, const KC &chain, const AnalyticalResult &raw, const VectorXd &reference)
{
    UnwrappedResult out;
    out.status       = raw.status;
    out.error_metric = raw.error_metric;
    if(raw.status != py_analytical_status::ok)
    {
        return out;
    }

    const int n      = chain.num_joints();
    const double tol = cartan::detail::default_feasibility_tol<double>();
    out.solutions.reserve(raw.solutions.size());
    out.tags.reserve(raw.solutions.size());
    for(const auto &q : raw.solutions)
    {
        if(q.size() != n)
        {
            std::ostringstream s;
            s << fn_name << ": solver returned solution size " << q.size() << " for chain.num_joints() " << n;
            throw nb::value_error(s.str().c_str());
        }
        VectorXd unwrapped = unwrap_solution(chain, q, reference);
        out.tags.push_back(cartan::detail::within_limits(unwrapped, chain, tol) ? cartan::range_status::in_range : cartan::range_status::joint_limits_violated);
        out.solutions.push_back(std::move(unwrapped));
    }
    return out;
}

inline AnalyticalResult solve_opw_result(const KC &chain, const OPWParametersd &params, const SE3d &target, double position_tolerance, double singularity_tolerance)
{
    auto solver = cartan::opw_6r_solver<KC>::make(chain, params, position_tolerance, singularity_tolerance);
    if(!solver)
    {
        return to_analytical_error_result(solver.error());
    }
    return to_analytical_result(solver->solve(target));
}

inline AnalyticalResult solve_pieper_result(const KC &chain, const SE3d &target)
{
    cartan::pieper_6r_solver solver(chain);
    return to_analytical_result(solver.solve(target));
}

inline AnalyticalResult solve_planar_result(const KC &chain, const SE3d &target)
{
    cartan::planar_2r_solver solver(chain);
    return to_analytical_result(solver.solve(target));
}

inline AnalyticalResult solve_spatial_3r_result(const KC &chain, const SE3d &target)
{
    cartan::spatial_3r_solver solver(chain);
    return to_analytical_result(solver.solve(target));
}

}

namespace cartan::python {

void register_analytical(nb::module_ &m)
{
    nb::module_ analytical = m.def_submodule("analytical", "Closed-form analytical IK solvers and Paden-Kahan subproblems.");

    // ------------------------------------------------------------------
    // AnalyticalStatus enum (5 variants mirroring C++ analytical_failure
    // plus a Python success sentinel "ok"). The set is fixed to mirror
    // cartan::analytical_failure; the Python surface intentionally does
    // not expose any legacy "singular" / "near_singular" names.
    // ------------------------------------------------------------------
    nb::enum_<py_analytical_status>(analytical, "AnalyticalStatus",
                                    "Outcome of an analytical IK solve. ok signals success; the four "
                                    "failure variants mirror cartan::analytical_failure.")
            .value("ok", py_analytical_status::ok)
            .value("unreachable", py_analytical_status::unreachable)
            .value("degenerate_geometry", py_analytical_status::degenerate_geometry)
            .value("singular_configuration", py_analytical_status::singular_configuration)
            .value("verification_failed", py_analytical_status::verification_failed);

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
            .def_ro("solutions", &AnalyticalResult::solutions)
            .def_ro("status", &AnalyticalResult::status)
            .def_ro("error_metric", &AnalyticalResult::error_metric)
            .def("__repr__",
                 [](const AnalyticalResult &r)
                 {
                     std::ostringstream s;
                     s << "AnalyticalResult(num_solutions=" << r.solutions.size() << ", status=" << static_cast<int>(r.status) << ", error_metric=" << r.error_metric << ")";
                     return s.str();
                 });

    nb::class_<OPWParametersd>(analytical, "OPWParameters", "Geometric OPW parameters for an ortho-parallel spherical-wrist 6R arm.")
            .def(
                    "__init__",
                    [](OPWParametersd *self, double a1, double a2, double b, double c1, double c2, double c3, double c4, const std::vector<double> &offsets,
                       const std::vector<int> &sign_corrections) { new(self) OPWParametersd(make_opw_parameters(a1, a2, b, c1, c2, c3, c4, offsets, sign_corrections)); },
                    "Construct OPW parameters from seven lengths, six offsets, and "
                    "six sign corrections. sign_corrections entries must be -1 or +1.",
                    nb::arg("a1"), nb::arg("a2"), nb::arg("b"), nb::arg("c1"), nb::arg("c2"), nb::arg("c3"), nb::arg("c4"), nb::arg("offsets"), nb::arg("sign_corrections"))
            .def_rw("a1", &OPWParametersd::a1)
            .def_rw("a2", &OPWParametersd::a2)
            .def_rw("b", &OPWParametersd::b)
            .def_rw("c1", &OPWParametersd::c1)
            .def_rw("c2", &OPWParametersd::c2)
            .def_rw("c3", &OPWParametersd::c3)
            .def_rw("c4", &OPWParametersd::c4)
            .def_prop_rw("offsets", &opw_offsets, &set_opw_offsets)
            .def_prop_rw("sign_corrections", &opw_sign_corrections, &set_opw_sign_corrections)
            .def("__repr__",
                 [](const OPWParametersd &p)
                 {
                     std::ostringstream s;
                     s << "OPWParameters(a1=" << p.a1 << ", a2=" << p.a2 << ", b=" << p.b << ", c1=" << p.c1 << ", c2=" << p.c2 << ", c3=" << p.c3 << ", c4=" << p.c4 << ")";
                     return s.str();
                 });

    nb::enum_<cartan::opw_branch>(analytical, "OPWBranch", "Stable OPW branch key in ascending shoulder/elbow/wrist order.")
            .value("front_up_no_flip", cartan::opw_branch::front_up_no_flip)
            .value("front_up_flip", cartan::opw_branch::front_up_flip)
            .value("front_down_no_flip", cartan::opw_branch::front_down_no_flip)
            .value("front_down_flip", cartan::opw_branch::front_down_flip)
            .value("back_up_no_flip", cartan::opw_branch::back_up_no_flip)
            .value("back_up_flip", cartan::opw_branch::back_up_flip)
            .value("back_down_no_flip", cartan::opw_branch::back_down_no_flip)
            .value("back_down_flip", cartan::opw_branch::back_down_flip);

    nb::enum_<cartan::range_status>(analytical, "RangeStatus", "Per-solution range verdict for an unwrapped analytical solution.")
            .value("in_range", cartan::range_status::in_range)
            .value("joint_limits_violated", cartan::range_status::joint_limits_violated);

    nb::class_<UnwrappedResult>(analytical, "UnwrappedResult", "Closed-form solve outcome with per-solution range tags.")
            .def_ro("solutions", &UnwrappedResult::solutions)
            .def_ro("tags", &UnwrappedResult::tags)
            .def_ro("status", &UnwrappedResult::status)
            .def_ro("error_metric", &UnwrappedResult::error_metric)
            .def("__repr__",
                 [](const UnwrappedResult &r)
                 {
                     std::ostringstream s;
                     s << "UnwrappedResult(num_solutions=" << r.solutions.size() << ", status=" << static_cast<int>(r.status) << ", error_metric=" << r.error_metric << ")";
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
    analytical.def(
            "solve_pieper_6r",
            [](const KC &chain, const SE3d &target) -> AnalyticalResult
            {
                validate_target_finite("solve_pieper_6r", target);
                cartan::pieper_6r_solver solver(chain);
                return to_analytical_result(solver.solve(target));
            },
            "Closed-form 6R inverse kinematics for Pieper-type wrists. "
            "Returns up to 8 FK-verified branches; on a non-Pieper chain "
            "status is degenerate_geometry and solutions is empty.",
            nb::arg("chain"), nb::arg("target").noconvert(), nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "solve_planar_2r",
            [](const KC &chain, const SE3d &target) -> AnalyticalResult
            {
                validate_target_finite("solve_planar_2r", target);
                cartan::planar_2r_solver solver(chain);
                return to_analytical_result(solver.solve(target));
            },
            "Closed-form planar 2R inverse kinematics. Returns up to two "
            "FK-verified branches (elbow-up and elbow-down).",
            nb::arg("chain"), nb::arg("target").noconvert(), nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "solve_3r",
            [](const KC &chain, const SE3d &target) -> AnalyticalResult
            {
                validate_target_finite("solve_3r", target);
                cartan::spatial_3r_solver solver(chain);
                return to_analytical_result(solver.solve(target));
            },
            "Closed-form spatial 3R position-only inverse kinematics. "
            "Returns up to four FK-verified branches.",
            nb::arg("chain"), nb::arg("target").noconvert(), nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "solve_opw_6r",
            [](const KC &chain, const OPWParametersd &params, const SE3d &target, double position_tolerance, double singularity_tolerance) -> AnalyticalResult
            {
                validate_target_finite("solve_opw_6r", target);
                nb::gil_scoped_release release;
                return solve_opw_result(chain, params, target, position_tolerance, singularity_tolerance);
            },
            "Closed-form OPW inverse kinematics for offset-shoulder, "
            "ortho-parallel, spherical-wrist 6R arms. Returns up to 8 "
            "FK-verified branches through the same AnalyticalResult contract as "
            "the other analytical solvers.",
            nb::arg("chain"), nb::arg("params"), nb::arg("target").noconvert(), nb::arg("position_tolerance") = 1e-9, nb::arg("singularity_tolerance") = 1e-9);

    analytical.def(
            "solve_unwrapped_opw_6r",
            [](const KC &chain, const OPWParametersd &params, const SE3d &target, std::optional<nb::DRef<const VectorXd>> q_seed, double position_tolerance,
               double singularity_tolerance) -> UnwrappedResult
            {
                validate_target_finite("solve_unwrapped_opw_6r", target);
                VectorXd reference = validated_reference("solve_unwrapped_opw_6r", chain, q_seed);
                nb::gil_scoped_release release;
                AnalyticalResult raw = solve_opw_result(chain, params, target, position_tolerance, singularity_tolerance);
                return unwrap_analytical_result("solve_unwrapped_opw_6r", chain, raw, reference);
            },
            "Solve OPW IK and return every branch with a per-solution range tag. "
            "q_seed selects the nearest 2*pi representative when a joint range "
            "spans multiple turns.",
            nb::arg("chain"), nb::arg("params"), nb::arg("target").noconvert(), nb::kw_only(), nb::arg("q_seed") = nb::none(), nb::arg("position_tolerance") = 1e-9,
            nb::arg("singularity_tolerance") = 1e-9);

    analytical.def(
            "solve_unwrapped_pieper_6r",
            [](const KC &chain, const SE3d &target, std::optional<nb::DRef<const VectorXd>> q_seed) -> UnwrappedResult
            {
                validate_target_finite("solve_unwrapped_pieper_6r", target);
                VectorXd reference = validated_reference("solve_unwrapped_pieper_6r", chain, q_seed);
                nb::gil_scoped_release release;
                AnalyticalResult raw = solve_pieper_result(chain, target);
                return unwrap_analytical_result("solve_unwrapped_pieper_6r", chain, raw, reference);
            },
            "Solve Pieper 6R IK and return every branch with a per-solution "
            "range tag.",
            nb::arg("chain"), nb::arg("target").noconvert(), nb::kw_only(), nb::arg("q_seed") = nb::none());

    analytical.def(
            "solve_unwrapped_3r",
            [](const KC &chain, const SE3d &target, std::optional<nb::DRef<const VectorXd>> q_seed) -> UnwrappedResult
            {
                validate_target_finite("solve_unwrapped_3r", target);
                VectorXd reference = validated_reference("solve_unwrapped_3r", chain, q_seed);
                nb::gil_scoped_release release;
                AnalyticalResult raw = solve_spatial_3r_result(chain, target);
                return unwrap_analytical_result("solve_unwrapped_3r", chain, raw, reference);
            },
            "Solve spatial 3R IK and return every branch with a per-solution "
            "range tag.",
            nb::arg("chain"), nb::arg("target").noconvert(), nb::kw_only(), nb::arg("q_seed") = nb::none());

    analytical.def(
            "solve_unwrapped_planar_2r",
            [](const KC &chain, const SE3d &target, std::optional<nb::DRef<const VectorXd>> q_seed) -> UnwrappedResult
            {
                validate_target_finite("solve_unwrapped_planar_2r", target);
                VectorXd reference = validated_reference("solve_unwrapped_planar_2r", chain, q_seed);
                nb::gil_scoped_release release;
                AnalyticalResult raw = solve_planar_result(chain, target);
                return unwrap_analytical_result("solve_unwrapped_planar_2r", chain, raw, reference);
            },
            "Solve planar 2R IK and return every branch with a per-solution "
            "range tag.",
            nb::arg("chain"), nb::arg("target").noconvert(), nb::kw_only(), nb::arg("q_seed") = nb::none());

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
    analytical.def(
            "paden_kahan_1",
            [](const Vec3d &omega, const Vec3d &q, const Vec3d &p, const Vec3d &p_prime) -> std::optional<double>
            {
                auto r = cartan::paden_kahan_1<double>(omega, q, p, p_prime);
                if(!r)
                    return std::nullopt;
                return *r;
            },
            "Paden-Kahan subproblem 1: find theta such that exp([omega]*theta) "
            "applied at q maps p to p'. Returns None when the constraint has no "
            "solution (the two points are not equidistant from the axis).",
            nb::arg("omega").noconvert(), nb::arg("q").noconvert(), nb::arg("p").noconvert(), nb::arg("p_prime").noconvert(), nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "paden_kahan_2",
            [](const Vec3d &omega1, const Vec3d &omega2, const Vec3d &q, const Vec3d &p, const Vec3d &p_prime) -> std::vector<std::pair<double, double>>
            {
                auto r = cartan::paden_kahan_2<double>(omega1, omega2, q, p, p_prime);
                std::vector<std::pair<double, double>> out;
                if(!r)
                    return out;
                out.reserve(static_cast<std::size_t>(r->count));
                for(int i = 0; i < r->count; ++i)
                {
                    out.emplace_back(r->solutions[static_cast<std::size_t>(i)]);
                }
                return out;
            },
            "Paden-Kahan subproblem 2: find up to two (theta1, theta2) pairs "
            "such that exp([omega1]*theta1) * exp([omega2]*theta2) applied at q "
            "maps p to p'. Axes omega1 and omega2 must intersect at q.",
            nb::arg("omega1").noconvert(), nb::arg("omega2").noconvert(), nb::arg("q").noconvert(), nb::arg("p").noconvert(), nb::arg("p_prime").noconvert(),
            nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "paden_kahan_3",
            [](const Vec3d &omega, const Vec3d &q, const Vec3d &p, const Vec3d &p_prime, double delta) -> std::vector<double>
            {
                auto r = cartan::paden_kahan_3<double>(omega, q, p, p_prime, delta);
                std::vector<double> out;
                if(!r)
                    return out;
                out.reserve(static_cast<std::size_t>(r->count));
                for(int i = 0; i < r->count; ++i)
                {
                    out.push_back(r->solutions[static_cast<std::size_t>(i)]);
                }
                return out;
            },
            "Paden-Kahan subproblem 3: find up to two theta values such that "
            "||exp([omega]*theta)*p - p'|| == delta, with the rotation taken "
            "about the axis omega through q.",
            nb::arg("omega").noconvert(), nb::arg("q").noconvert(), nb::arg("p").noconvert(), nb::arg("p_prime").noconvert(), nb::arg("delta"),
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
    analytical.def(
            "closest_to_seed",
            [](const AnalyticalResult &result, const nb::DRef<const VectorXd> &q_seed) -> std::optional<VectorXd>
            {
                if(result.solutions.empty())
                    return std::nullopt;
                const VectorXd seed = q_seed;
                auto it             = std::min_element(result.solutions.begin(), result.solutions.end(),
                                                       [&seed](const VectorXd &a, const VectorXd &b) { return l2_distance(a, seed) < l2_distance(b, seed); });
                return *it;
            },
            "Return the solution closest to q_seed by L2 distance. "
            "Returns None when result.solutions is empty.",
            nb::arg("result"), nb::arg("q_seed").noconvert(), nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "verify_solution",
            [](const KC &chain, const nb::DRef<const VectorXd> &q, const SE3d &target, double tolerance) -> bool
            {
                validate_target_finite("verify_solution", target);
                if(q.size() != chain.num_joints())
                {
                    std::ostringstream s;
                    s << "verify_solution: q.size() (" << q.size() << ") does not match chain.num_joints() (" << chain.num_joints() << ")";
                    throw nb::value_error(s.str().c_str());
                }
                cartan::convergence_criteria<double> criteria{tolerance, tolerance, 0, 0};
                return cartan::verify_solution(chain, target, VectorXd(q), criteria);
            },
            "FK back-check: forward_kinematics(chain, q) must agree with target "
            "to within tolerance in both translation and rotation components of "
            "the body twist. The single tolerance argument is applied to both "
            "the position and orientation tolerances of the C++ verifier.",
            nb::arg("chain"), nb::arg("q").noconvert(), nb::arg("target").noconvert(), nb::arg("tolerance") = 1e-6, nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "filter_valid_solutions",
            [](const KC &chain, const AnalyticalResult &result, const SE3d &target, double tolerance) -> AnalyticalResult
            {
                validate_target_finite("filter_valid_solutions", target);
                cartan::convergence_criteria<double> criteria{tolerance, tolerance, 0, 0};
                AnalyticalResult filtered;
                filtered.status       = result.status;
                filtered.error_metric = result.error_metric;
                filtered.solutions.reserve(result.solutions.size());
                for(const auto &q : result.solutions)
                {
                    if(q.size() != chain.num_joints())
                        continue;
                    if(cartan::verify_solution(chain, target, q, criteria))
                    {
                        filtered.solutions.push_back(q);
                    }
                }
                return filtered;
            },
            "Return a new AnalyticalResult keeping only the solutions that "
            "FK-verify against target within tolerance. The status and "
            "error_metric of the input are preserved.",
            nb::arg("chain"), nb::arg("result"), nb::arg("target").noconvert(), nb::arg("tolerance") = 1e-6, nb::call_guard<nb::gil_scoped_release>());

    analytical.def(
            "solve_all",
            [](const KC &chain, const SE3d &target, std::optional<nb::DRef<const VectorXd>> q_seed, int /*rank*/) -> AnalyticalResult
            {
                validate_target_finite("solve_all", target);
                AnalyticalResult result;
                const int n = chain.num_joints();
                switch(n)
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
                        result.status       = py_analytical_status::degenerate_geometry;
                        result.error_metric = 0.0;
                        return result;
                }

                if(q_seed.has_value() && !result.solutions.empty())
                {
                    const VectorXd seed = q_seed.value();
                    std::sort(result.solutions.begin(), result.solutions.end(), [&seed](const VectorXd &a, const VectorXd &b) { return l2_distance(a, seed) < l2_distance(b, seed); });
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
            nb::arg("chain"), nb::arg("target").noconvert(), nb::kw_only(), nb::arg("q_seed") = nb::none(), nb::arg("rank") = 0, nb::call_guard<nb::gil_scoped_release>());
}

}
