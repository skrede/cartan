#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_DETAIL_HALTON_SEED_GENERATOR_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_DETAIL_HALTON_SEED_GENERATOR_H

/// Halton sequence seed generator with Beeson-Ames joint wrapping.
///
/// Produces deterministic low-discrepancy seed configurations for multi-start
/// IK by mapping Halton sequences from [0,1]^N to joint limit ranges. Uses
/// Beeson-Ames wrapping to handle past-limit joints via modulo 2pi with
/// fallback to clamping.
///
/// Reference: Beeson & Ames, "TRAC-IK: An Open-Source Library for Improved
/// Solving of Generic Inverse Kinematics", 2015.

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/chain_concept.h"

#include <array>
#include <cmath>
#include <numbers>

namespace cartan
{

/// Compute a single element of the Halton (van der Corput) sequence.
template <typename Scalar>
[[nodiscard]] constexpr Scalar halton_element(int index, int base)
{
    Scalar result{0};
    Scalar f = Scalar(1) / static_cast<Scalar>(base);
    int i = index;
    while (i > 0)
    {
        result += f * static_cast<Scalar>(i % base);
        i /= base;
        f /= static_cast<Scalar>(base);
    }
    return result;
}

/// Beeson-Ames joint angle wrapping.
///
/// If q is outside [q_min, q_max], wraps modulo 2pi towards the violated
/// limit. Falls back to clamping when the wrapped value lands outside the
/// opposite limit (occurs when the joint range is less than 2pi).
template <typename Scalar>
[[nodiscard]] Scalar wrap_joint_angle(Scalar q, Scalar q_min, Scalar q_max)
{
    constexpr Scalar two_pi = Scalar(2) * std::numbers::pi_v<Scalar>;

    if (q >= q_min && q <= q_max)
        return q;

    if (q < q_min)
    {
        Scalar diff = std::fmod(q_min - q, two_pi);
        Scalar wrapped = q_min - diff + two_pi;
        return (wrapped > q_max) ? q_min : wrapped;
    }

    // q > q_max
    Scalar diff = std::fmod(q - q_max, two_pi);
    Scalar wrapped = q_max + diff - two_pi;
    return (wrapped < q_min) ? q_max : wrapped;
}

/// Deterministic low-discrepancy seed generator for multi-start IK.
///
/// Uses Halton sequences (one per joint dimension, each with a distinct
/// prime base) to generate seed configurations that uniformly cover the
/// joint space. The first `skip_count` entries are dropped to reduce
/// initial correlation between dimensions.
template <chain Chain>
class halton_seed_generator
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;

public:
    using position_type = typename joint_state<Scalar, N>::position_type;

    /// First 10 primes for Halton bases (supports up to 10 joints).
    static constexpr std::array<int, 10> bases = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29};

    /// Drop first 20 entries to reduce initial correlation between dimensions.
    static constexpr int skip_count = 20;

    /// Construct from a chain (borrows reference; chain must outlive generator).
    explicit halton_seed_generator(const Chain& chain)
        : m_chain(&chain)
    {
    }

    /// Generate seed configuration for the given restart index (0-based).
    ///
    /// Maps Halton [0,1]^N to joint limits via linear scaling.
    [[nodiscard]] position_type operator()(int index) const
    {
        int n = m_chain->num_joints();
        position_type q;
        if constexpr (N == dynamic)
            q.resize(n);

        int halton_index = index + skip_count + 1; // +1 because halton_element(0) = 0

        for (int j = 0; j < n; ++j)
        {
            Scalar h = halton_element<Scalar>(halton_index, bases[static_cast<std::size_t>(j)]);
            auto lim = m_chain->limits()[static_cast<std::size_t>(j)];
            // Scale [0,1] -> [q_min, q_max] for finite-range joints; for an
            // unbounded angular joint, fall back to one principal revolution
            // centered at zero so the seed remains finite.
            const Scalar range = lim.position_max - lim.position_min;
            if (std::isfinite(range))
            {
                q[j] = lim.position_min + h * range;
            }
            else
            {
                q[j] = (h - Scalar(0.5)) * cartan::detail::k_unbounded_angular_range_v<Scalar>;
            }
        }
        return q;
    }

private:
    const Chain* m_chain;
};

}

#endif
