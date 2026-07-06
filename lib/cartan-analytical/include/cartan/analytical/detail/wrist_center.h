#ifndef HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_WRIST_CENTER_H
#define HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_WRIST_CENTER_H

#include "cartan/analytical/analytical_types.h"

#include "cartan/serial/chain/screw_axis.h"

#include "cartan/lie/se3.h"
#include "cartan/detail/epsilon.h"
#include "cartan/types.h"

#include <cmath>
#include "cartan/expected.h"

namespace cartan::detail
{

/// Compute wrist center position from target pose and tool offset.
///
/// p_wrist = p_target - R_target * d_tool
///
/// The tool offset is the vector from wrist center to end-effector
/// expressed in the tool (body) frame.
template <typename Scalar>
vector3<Scalar> compute_wrist_center(
    const se3<Scalar>& target,
    const vector3<Scalar>& tool_offset)
{
    return target.translation() - target.rotation().act(tool_offset);
}

/// Closest approach point of two lines in 3D.
///
/// Line i: q_i + t * d_i. Returns the midpoint of the closest approach
/// segment. If lines are parallel, returns q1.
template <typename Scalar>
vector3<Scalar> closest_approach_midpoint(
    const vector3<Scalar>& q1, const vector3<Scalar>& d1,
    const vector3<Scalar>& q2, const vector3<Scalar>& d2)
{
    vector3<Scalar> w = q1 - q2;
    Scalar a = d1.dot(d1);
    Scalar b = d1.dot(d2);
    Scalar c = d2.dot(d2);
    Scalar e = d1.dot(w);
    Scalar f = d2.dot(w);

    Scalar denom = a * c - b * b;
    if (std::abs(denom) < epsilon_v<Scalar>)
        return q1;

    Scalar t = (b * f - c * e) / denom;
    Scalar s = (a * f - b * e) / denom;

    vector3<Scalar> p1 = q1 + t * d1;
    vector3<Scalar> p2 = q2 + s * d2;
    return (p1 + p2) / Scalar(2);
}

/// Closest approach distance between two lines in 3D.
template <typename Scalar>
Scalar closest_approach_distance(
    const vector3<Scalar>& q1, const vector3<Scalar>& d1,
    const vector3<Scalar>& q2, const vector3<Scalar>& d2)
{
    vector3<Scalar> cross = d1.cross(d2);
    Scalar cross_norm = cross.norm();

    if (cross_norm < sqrt_epsilon_v<Scalar>)
    {
        // Parallel lines: distance = |(q2-q1) x d1| / |d1|
        return (q2 - q1).cross(d1).norm() / d1.norm();
    }

    return std::abs((q2 - q1).dot(cross)) / cross_norm;
}

/// Validate that three revolute joint axes approximately intersect at a
/// common point. Returns the intersection point or error if axes do not
/// intersect within tolerance.
///
/// For each pair of axes, computes the closest approach distance. If all
/// three pairwise distances are below tolerance, the intersection point
/// is the average of the three pairwise closest points.
template <typename Scalar>
cartan::expected<vector3<Scalar>, analytical_failure>
find_wrist_intersection(
    const screw_axis<Scalar>& axis4,
    const screw_axis<Scalar>& axis5,
    const screw_axis<Scalar>& axis6,
    Scalar tolerance = Scalar(1e-3))
{
    // Extract line representations: point on axis = omega x v / |omega|^2
    // For unit omega, point = omega x v
    vector3<Scalar> q4 = axis4.omega().cross(axis4.v());
    vector3<Scalar> q5 = axis5.omega().cross(axis5.v());
    vector3<Scalar> q6 = axis6.omega().cross(axis6.v());
    vector3<Scalar> d4 = axis4.omega();
    vector3<Scalar> d5 = axis5.omega();
    vector3<Scalar> d6 = axis6.omega();

    // Check pairwise distances and collect intersection points from
    // non-parallel pairs. Parallel/identical axes cannot produce a
    // unique closest point, so they are skipped for averaging.
    struct pair_info { Scalar distance; vector3<Scalar> point; bool parallel; };
    auto compute_pair = [&](
        const vector3<Scalar>& qa, const vector3<Scalar>& da,
        const vector3<Scalar>& qb, const vector3<Scalar>& db) -> pair_info
    {
        Scalar cross_norm = da.cross(db).norm();
        bool par = cross_norm < sqrt_epsilon_v<Scalar>;
        Scalar dist = closest_approach_distance(qa, da, qb, db);
        vector3<Scalar> pt = par ? qa : closest_approach_midpoint(qa, da, qb, db);
        return {dist, pt, par};
    };

    auto p45 = compute_pair(q4, d4, q5, d5);
    auto p46 = compute_pair(q4, d4, q6, d6);
    auto p56 = compute_pair(q5, d5, q6, d6);

    // All pairwise distances must be within tolerance
    if (p45.distance > tolerance || p46.distance > tolerance || p56.distance > tolerance)
        return cartan::unexpected(analytical_failure::degenerate_geometry);

    // Average only over non-parallel pairs
    vector3<Scalar> sum = vector3<Scalar>::Zero();
    int count = 0;
    if (!p45.parallel) { sum += p45.point; ++count; }
    if (!p46.parallel) { sum += p46.point; ++count; }
    if (!p56.parallel) { sum += p56.point; ++count; }

    if (count == 0)
    {
        // All three axes are parallel -- they cannot intersect at a point
        // unless they are all identical, in which case any point suffices.
        // This is a degenerate configuration for a wrist.
        return cartan::unexpected(analytical_failure::degenerate_geometry);
    }

    return sum / Scalar(count);
}

}

#endif
