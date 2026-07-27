#ifndef HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H
#define HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H

#include "cartan/serial/chain/joint_state.h"

#include <array>
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
    singular_configuration,  ///< Mechanism is at a kinematic singularity for the requested target.
    verification_failed,     ///< Candidate solutions exist but none survived the FK back-check.
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
        return "Mechanism is at a kinematic singularity for the requested target";
    case analytical_failure::verification_failed:
        return "No candidate solution survived the forward-kinematics back-check";
    case analytical_failure::non_finite_input:
        return "Input contains a NaN or infinite component";
    }
    return "Unknown analytical_failure";
}

/// Failure diagnostic for analytical solvers. `reason` names the failure mode;
/// `workspace_distance` is the magnitude (in the chain's linear unit) by which
/// the target exceeds the reachable workspace when `reason` is
/// `analytical_failure::unreachable`, and zero otherwise.
template <typename Scalar>
struct analytical_error
{
    analytical_failure reason;
    Scalar workspace_distance{};
};

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
template <typename Scalar>
class verification_tolerance
{
public:
    constexpr verification_tolerance(Scalar position, Scalar orientation)
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

template <typename Scalar>
inline constexpr verification_tolerance<Scalar> default_verification_tolerance_v{
    Scalar(1e-6), Scalar(1e-6)};

}

#endif
