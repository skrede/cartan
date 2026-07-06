#ifndef HPP_GUARD_CARTAN_LIE_LIE_FAILURE_H
#define HPP_GUARD_CARTAN_LIE_LIE_FAILURE_H

namespace cartan
{

/// Failure modes for the core Lie-group value factories. Allocation-free and
/// matchable; the sole error channel shared by so2/so3/se2/se3 from_matrix,
/// so3 from_quaternion, the frame-tagged rotation/transform wrappers, and
/// screw_axis::from_vector.
enum class lie_failure
{
    non_orthogonal,       ///< R^T * R deviates from identity (so2/so3 from_matrix).
    improper_rotation,    ///< det(R) != 1, a reflection rather than a rotation (so2/so3 from_matrix).
    non_unit_quaternion,  ///< ||q||^2 deviates from 1 (so3 from_quaternion).
    invalid_affine_row,   ///< Homogeneous bottom row is not [0..0 1] (se2/se3 from_matrix).
    non_unit_screw_axis   ///< Revolute ||omega|| != 1 or prismatic ||v|| != 1 (screw_axis from_vector).
};

/// Human-readable diagnostic for a lie_failure, for logging and binding
/// exception messages. Returns a static string literal; no allocation.
[[nodiscard]] constexpr const char* message(lie_failure failure)
{
    switch (failure)
    {
    case lie_failure::non_orthogonal:
        return "Matrix is not orthogonal: R^T * R deviates from identity";
    case lie_failure::improper_rotation:
        return "Matrix has determinant != 1: not a proper rotation";
    case lie_failure::non_unit_quaternion:
        return "Quaternion is not unit: ||q||^2 deviates from 1";
    case lie_failure::invalid_affine_row:
        return "Homogeneous bottom row is not [0..0 1]";
    case lie_failure::non_unit_screw_axis:
        return "Screw axis is not unit: revolute ||omega|| != 1 or prismatic ||v|| != 1";
    }
    return "Unknown lie_failure";
}

}

#endif
