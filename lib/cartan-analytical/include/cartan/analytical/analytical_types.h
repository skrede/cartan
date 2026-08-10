#ifndef HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H
#define HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H

#include "cartan/serial/chain/joint_state.h"

#include <array>
#include <optional>
#include "cartan/expected.h"

namespace cartan
{

/// Failure modes for the analytical IK solvers.
///
/// `non_finite_input` carries the same name here as on the chain and
/// inverse-kinematics failure enums, because it names the same defect: a NaN or
/// an infinity crossing a boundary that has no answer for one.
enum class analytical_failure
{
    unreachable,             ///< Target lies outside the mechanism's workspace.
    degenerate_geometry,     ///< Joint geometry violates a subproblem precondition (e.g. parallel axes where intersection is required).
    singular_configuration,  ///< Mechanism is at a kinematic singularity for the requested target, or the decomposition broke down and placed no candidate.
    verification_failed,     ///< Candidates were constructed and every one was rejected by the FK back-check.
    non_finite_input         ///< An input or candidate joint value is NaN or infinite.
};

/// Human-readable diagnostic for an analytical_failure, for logging and binding
/// exception messages. Returns a static string literal; no allocation.
constexpr const char* message(analytical_failure failure)
{
    switch (failure)
    {
    case analytical_failure::unreachable:
        return "Target lies outside the mechanism's workspace";
    case analytical_failure::degenerate_geometry:
        return "Joint geometry violates a subproblem precondition";
    case analytical_failure::singular_configuration:
        return "Mechanism is at a kinematic singularity, or the decomposition placed no candidate";
    case analytical_failure::verification_failed:
        return "No candidate solution survived the forward-kinematics back-check";
    case analytical_failure::non_finite_input:
        return "Input contains a NaN or infinite component";
    }
    return "Unknown analytical_failure";
}

/// Failure diagnostic for analytical solvers. `reason` names the failure mode.
/// `workspace_distance` is present only where a geometric inequality was
/// evaluated and failed, and is then a deficit measured at such an inequality in
/// the chain's linear unit; it is absent for every other failure. Absence is not
/// zero: a target sitting exactly on the workspace boundary has a deficit of
/// zero, so zero cannot also stand for "no magnitude was computed".
///
/// Where several inequalities were evaluated the value is a lower bound on the
/// motion the target needs, not a figure attached to `reason`: inequalities that
/// must all hold contribute the largest of their deficits, alternatives the
/// smallest of theirs, and a later check that measures nothing does not discard
/// what an earlier one measured.
template <typename Scalar>
struct analytical_error
{
    analytical_failure reason;
    std::optional<Scalar> workspace_distance;
};

/// Build a diagnostic around a reason a subproblem decided. Whichever reason it
/// is, the inequality that failed is inside the subproblem, whose error channel
/// carries no payload, so the reason travels alone: a caller substituting some
/// length it happens to have to hand would report a magnitude in the right unit
/// at an inequality nothing evaluated, which is worse than reporting none.
template <typename Scalar>
analytical_error<Scalar> subproblem_error(analytical_failure reason)
{
    return analytical_error<Scalar>{reason, std::nullopt};
}

/// Multi-solution result for an analytical IK solver. N is the joint count
/// (compile-time); MaxSolutions is the per-solver upper bound on solution
/// count (e.g. 2 for planar_2r_solver, 4 for spatial_3r_solver, 8 for
/// pieper_6r_solver). The `solutions` array is sized to MaxSolutions; only
/// the first `count` entries are populated and FK-verified. Use the
/// begin()/end() iterators to traverse the populated subset.
template <typename Scalar, int N, int MaxSolutions>
struct analytical_result
{
    using position_type = Eigen::Vector<Scalar, N>;

    std::array<position_type, static_cast<std::size_t>(MaxSolutions)> solutions;
    int count{0};

    auto begin() const { return solutions.begin(); }
    auto end() const { return solutions.begin() + count; }
};

/// Acceptance threshold for a residual between positions, applied *relative* to
/// the working radius: a residual is compared against value() scaled by the
/// larger of the two displacement norms and one. Round-off in a position
/// residual grows with that radius, so an absolute threshold would tighten as
/// the chain grows and reject correct answers on a large mechanism.
///
/// The constructor is explicit and the value private so that a braced scalar
/// cannot stand in for one of these where the other threshold type belongs.
template <typename Scalar>
class length_tolerance
{
public:
    constexpr explicit length_tolerance(Scalar value)
        : m_value(value)
    {
    }

    constexpr Scalar value() const { return m_value; }

private:
    Scalar m_value;
};

/// Acceptance threshold for a dimensionless residual between unit directions.
/// The relative scaling above degenerates to a factor of exactly one on unit
/// arguments, so this threshold is absolute. Not interchangeable with
/// length_tolerance: no conversion exists, so mixing the two is a compile error.
template <typename Scalar>
class direction_tolerance
{
public:
    constexpr explicit direction_tolerance(Scalar value)
        : m_value(value)
    {
    }

    constexpr Scalar value() const { return m_value; }

private:
    Scalar m_value;
};

/// position() is a distance in the chain's linear unit; orientation() is the
/// norm of the residual rotation vector, in radians.
///
/// The constructor is explicit, like the two single-threshold classes above, so
/// that a bare braced pair cannot appear at a call site: the two fields are in
/// different units and `{a, b}` shows the reader neither which is which nor that
/// the argument is a tolerance at all.
template <typename Scalar>
class verification_tolerance
{
public:
    constexpr explicit verification_tolerance(Scalar position, Scalar orientation)
        : m_position(position)
        , m_orientation(orientation)
    {
    }

    constexpr Scalar position() const { return m_position; }

    constexpr Scalar orientation() const { return m_orientation; }

private:
    Scalar m_position;
    Scalar m_orientation;
};

/// One part per million of the working radius, and one micrometer at unit
/// radius if that radius is read in meters.
template <typename Scalar>
inline constexpr length_tolerance<Scalar> default_length_tolerance_v{Scalar(1e-6)};

/// Absolute, and calibrated on the Pieper wrist decomposition, whose worst
/// accepted residual is about 7e-12 at double precision and reaches this value
/// itself at float.
template <typename Scalar>
inline constexpr direction_tolerance<Scalar> default_direction_tolerance_v{Scalar(1e-6)};

/// Both fields are read by the forward-kinematics back-check and by the
/// Paden-Kahan subproblem gates, so the value has to clear the reconstruction
/// error of the scalar it is applied at.
///
/// 1e-6 is about eight float epsilons, below what a float forward map can
/// reconstruct, so a target the mechanism reaches is reported as a whole-solve
/// failure: over 1024 reachable poses the ortho-parallel solver loses 51 whole
/// solves at float and the Pieper solver 23, where the same sweep at double
/// loses none. The unverified ortho-parallel variant answers all 1024 at float,
/// which places the loss at the gate rather than in the arithmetic. 1e-4 is the
/// smallest decade at which the float sweep refuses exactly the poses the double
/// sweep refuses at the same value; wider only widens the subproblem gates.
template <typename Scalar>
inline constexpr verification_tolerance<Scalar> default_verification_tolerance_v{
    Scalar(1e-6), Scalar(1e-6)};

template <>
inline constexpr verification_tolerance<float> default_verification_tolerance_v<float>{
    1e-4f, 1e-4f};

}

#endif
