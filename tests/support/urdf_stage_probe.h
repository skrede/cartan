#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_URDF_STAGE_PROBE_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_URDF_STAGE_PROBE_H

/// Field-by-field comparison of two parsed_model values staged from the same
/// document by different front halves.
///
/// Scalars are compared by bit pattern rather than by ==, because +0.0 == -0.0
/// is true and a sign that flipped between the two paths would otherwise pass
/// unseen. A NaN on both sides is counted as agreement before the bit
/// comparison runs: != would call two agreeing NaNs a difference, and two NaNs
/// that agree carry no obligation to carry the same payload.
///
/// Joints are compared positionally rather than as a set. The extractor
/// consumes them in staged order, so a reordering yields a different chain from
/// identical values.

#include <cartan/urdf.h>

#include <bit>
#include <cmath>
#include <string>
#include <vector>
#include <cstdint>
#include <optional>
#include <algorithm>
#include <type_traits>
#include <string_view>

namespace cartan::testing
{

/// compared counts every field the two models were asked about; numeric counts
/// the scalars alone, which is the subset a value-by-value measurement covers.
/// differences names the joint and the field, so a red run points at a record
/// rather than reporting that some number moved.
struct stage_tally
{
    std::size_t equal;
    std::size_t joints;
    std::string fixture;
    std::size_t numeric;
    std::size_t compared;
    double worst_difference;
    std::size_t nan_agreements;
    std::size_t numeric_differences;
    std::vector<std::string> differences;
};

template <typename Scalar>
class stage_probe
{
    using pattern = std::conditional_t<sizeof(Scalar) == 4, std::uint32_t, std::uint64_t>;

public:
    explicit stage_probe(std::string fixture)
        : m_at("<robot>")
        , m_tally{0, 0, std::move(fixture), 0, 0, 0.0, 0, 0, {}}
    {
    }

    void enter(const std::string& record)
    {
        m_at = record;
    }

    void text(std::string_view field, const std::string& lhs, const std::string& rhs)
    {
        note(field, lhs == rhs);
    }

    void scalar(std::string_view field, Scalar lhs, Scalar rhs)
    {
        ++m_tally.numeric;
        if (std::isnan(lhs) && std::isnan(rhs))
        {
            ++m_tally.nan_agreements;
            note(field, true);
            return;
        }
        const bool same = std::bit_cast<pattern>(lhs) == std::bit_cast<pattern>(rhs);
        if (!same)
        {
            ++m_tally.numeric_differences;
            widen(lhs, rhs);
        }
        note(field, same);
    }

    void bound(const std::string& field, const std::optional<Scalar>& lhs,
               const std::optional<Scalar>& rhs)
    {
        note(field + " declared", lhs.has_value() == rhs.has_value());
        if (lhs && rhs) { scalar(field, *lhs, *rhs); }
    }

    const stage_tally& tally() const
    {
        return m_tally;
    }

private:
    std::string m_at;
    stage_tally m_tally;

    void widen(Scalar lhs, Scalar rhs)
    {
        if (!std::isfinite(lhs) || !std::isfinite(rhs)) { return; }
        const double gap = std::abs(static_cast<double>(lhs) - static_cast<double>(rhs));
        m_tally.worst_difference = std::max(m_tally.worst_difference, gap);
    }

    void note(std::string_view field, bool same)
    {
        ++m_tally.compared;
        if (same)
        {
            ++m_tally.equal;
            return;
        }
        m_tally.differences.push_back(m_at + " " + std::string(field));
    }
};

inline std::string kind_name(parsed_joint_kind kind)
{
    switch (kind)
    {
    case parsed_joint_kind::fixed:
        return "fixed";
    case parsed_joint_kind::revolute:
        return "revolute";
    case parsed_joint_kind::continuous:
        return "continuous";
    case parsed_joint_kind::prismatic:
        return "prismatic";
    }
    return "unrecognized";
}

template <typename Scalar>
void probe_placement(stage_probe<Scalar>& probe, const parsed_joint<Scalar>& lhs,
                     const parsed_joint<Scalar>& rhs)
{
    const matrix3<Scalar> left = lhs.origin.rotation().matrix();
    const matrix3<Scalar> right = rhs.origin.rotation().matrix();
    for (Eigen::Index i = 0; i < left.size(); ++i)
    {
        probe.scalar("origin rotation " + std::to_string(i), left(i), right(i));
    }
    for (Eigen::Index i = 0; i < 3; ++i)
    {
        const std::string at = std::to_string(i);
        probe.scalar("origin xyz " + at, lhs.origin.translation()(i), rhs.origin.translation()(i));
        probe.scalar("axis " + at, lhs.axis(i), rhs.axis(i));
    }
}

template <typename Scalar>
void probe_joint(stage_probe<Scalar>& probe, const parsed_joint<Scalar>& lhs,
                 const parsed_joint<Scalar>& rhs)
{
    probe.enter(lhs.name);
    probe.text("name", lhs.name, rhs.name);
    probe.text("kind", kind_name(lhs.kind), kind_name(rhs.kind));
    probe.text("parent link", lhs.parent_link, rhs.parent_link);
    probe.text("child link", lhs.child_link, rhs.child_link);
    probe_placement(probe, lhs, rhs);
    probe.bound("limit lower", lhs.position_min, rhs.position_min);
    probe.bound("limit upper", lhs.position_max, rhs.position_max);
    probe.bound("limit velocity", lhs.velocity_max, rhs.velocity_max);
    probe.bound("limit effort", lhs.effort_max, rhs.effort_max);
}

template <typename Scalar>
stage_tally probe_model(const std::string& fixture, const parsed_model<Scalar>& lhs,
                        const parsed_model<Scalar>& rhs)
{
    stage_probe<Scalar> probe(fixture);
    probe.text("robot name", lhs.robot_name, rhs.robot_name);
    probe.text("joint count", std::to_string(lhs.joints.size()), std::to_string(rhs.joints.size()));
    const std::size_t shared = std::min(lhs.joints.size(), rhs.joints.size());
    for (std::size_t i = 0; i < shared; ++i)
    {
        probe_joint(probe, lhs.joints[i], rhs.joints[i]);
    }
    stage_tally out = probe.tally();
    out.joints = shared;
    return out;
}

}

#endif
