#ifndef HPP_GUARD_CARTAN_TESTS_FUZZ_HARNESS_H
#define HPP_GUARD_CARTAN_TESTS_FUZZ_HARNESS_H

/// The primitives the fuzz targets here share: a cursor that reinterprets
/// fixed-width chunks of the fuzzer's byte stream as the values a factory
/// takes, the oracle a target states its contract through, and the sinks that
/// keep a produced value from being optimized away.

#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/serial/fk/forward_kinematics.h>

#include <span>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace cartan::fuzzing
{

/// Sequential decoder over the fuzzer's input.
///
/// A field beyond the end of the input reads as +0 rather than stopping the
/// harness, so a short input is the leading fields followed by zeros and the
/// empty input is the all-zero decode. Nothing is rejected or normalized on the
/// way out: a NaN, an infinity, a denormal and a zero are all values the
/// factories under test are supposed to answer for.
class field_reader
{
public:
    explicit field_reader(std::span<const std::uint8_t> input)
        : m_input(input)
        , m_offset(0)
    {
    }

    double scalar()
    {
        double value = 0.0;
        if (m_offset + sizeof(value) <= m_input.size())
        {
            std::memcpy(&value, m_input.data() + m_offset, sizeof(value));
        }
        m_offset += sizeof(value);
        return value;
    }

    std::uint8_t byte()
    {
        const std::uint8_t value = m_offset < m_input.size() ? m_input[m_offset] : 0;
        ++m_offset;
        return value;
    }

private:
    std::span<const std::uint8_t> m_input;
    std::size_t m_offset;
};

/// A target's oracle: the contract a value the library admitted must satisfy.
///
/// Without one a target asks only "did it crash?", and a boundary whose whole
/// job is arithmetic over doubles has nothing to crash with -- a weakened
/// predicate there would pass silently. abort() rather than assert() so the
/// check survives NDEBUG, and so the fuzzer retains the offending input as an
/// artifact instead of printing a line nobody reads.
inline void require(bool condition, const char* contract)
{
    if (condition)
    {
        return;
    }
    std::fprintf(stderr, "cartan fuzz oracle violated: %s\n", contract);
    std::abort();
}

inline volatile double observation_sink = 0.0;

/// Consume a value a factory produced. Without this the whole call chain behind
/// a successful construction is dead code the optimizer is free to delete, and
/// the sanitizers then have nothing to inspect.
inline void observe(double value)
{
    observation_sink = value;
}

/// Put a chain a factory produced to work, so a defect in the value cannot
/// escape by nobody touching it.
inline void consume(const cartan::kinematic_chain<double, cartan::dynamic>& chain)
{
    const Eigen::VectorXd q = Eigen::VectorXd::Zero(chain.num_joints());
    auto pose = cartan::forward_kinematics(chain, q);
    if (pose.has_value())
    {
        observe(pose.value().end_effector.translation().squaredNorm());
    }
}

}

#endif
