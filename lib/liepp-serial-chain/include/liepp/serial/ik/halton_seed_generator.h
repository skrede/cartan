#ifndef HPP_GUARD_LIEPP_SERIAL_IK_HALTON_SEED_GENERATOR_H
#define HPP_GUARD_LIEPP_SERIAL_IK_HALTON_SEED_GENERATOR_H

/// @file halton_seed_generator.h
/// @brief Halton sequence seed generator with Beeson-Ames joint wrapping.
///
/// Produces deterministic low-discrepancy seed configurations for multi-start
/// IK by mapping Halton sequences from [0,1]^N to joint limit ranges. Uses
/// Beeson-Ames wrapping to handle past-limit joints via modulo 2pi with
/// fallback to clamping.
///
/// Reference: Beeson & Ames, "TRAC-IK: An Open-Source Library for Improved
/// Solving of Generic Inverse Kinematics", 2015.

#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/joint_limits.h"
#include "liepp/serial/chain/kinematic_chain.h"

#include <array>
#include <cmath>
#include <numbers>

namespace liepp
{

/// Compute a single element of the Halton (van der Corput) sequence.
/// @param index  1-based sequence index (index 0 always returns 0).
/// @param base   Prime base for the sequence (2, 3, 5, 7, ...).
/// @return Value in (0, 1) for index > 0.
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
///
/// @param q      Joint angle to wrap.
/// @param q_min  Lower joint limit.
/// @param q_max  Upper joint limit.
/// @return Wrapped (or clamped) joint angle in [q_min, q_max].
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
///
/// @tparam Scalar Floating-point type.
/// @tparam N      Number of joints (compile-time) or liepp::dynamic.
template <typename Scalar = double, int N = dynamic>
class halton_seed_generator
{
public:
    using position_type = typename joint_state<Scalar, N>::position_type;

    /// First 10 primes for Halton bases (supports up to 10 joints).
    static constexpr std::array<int, 10> bases = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29};

    /// Drop first 20 entries to reduce initial correlation between dimensions.
    static constexpr int skip_count = 20;

    /// Construct from a kinematic chain (borrows reference; chain must outlive generator).
    explicit halton_seed_generator(const kinematic_chain<Scalar, N>& chain)
        : m_chain(&chain)
    {
    }

    /// Generate seed configuration for the given restart index (0-based).
    ///
    /// Maps Halton [0,1]^N to joint limits via linear scaling.
    /// @param index  0-based restart index.
    /// @return Joint position vector within the chain's joint limits.
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
            // Scale [0,1] -> [q_min, q_max]
            q[j] = lim.position_min + h * (lim.position_max - lim.position_min);
        }
        return q;
    }

private:
    const kinematic_chain<Scalar, N>* m_chain;
};

}

#endif
