#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_COUNTING_CHAIN_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_COUNTING_CHAIN_H

// Kernel-evaluation counting by chain substitution, for tests. A solver calls
// the unchecked forward-kinematics and body-Jacobian entry points unqualified,
// so argument-dependent lookup on this adaptor finds the overloads below and a
// solve counts its own kernel evaluations without knowing it is being counted.
//
// The measurement harness carries its own copy of this mechanism. This one is
// separate because tests verify cartan using nothing but cartan, and an
// instrument the tests depend on must not reach into the harness to exist.

#include <cartan/lie/se3.h>
#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <cstdint>

namespace cartan::testing
{

struct kernel_counts
{
    std::int64_t fk;
    std::int64_t jac;
};

template <typename Inner>
class counting_chain
{
public:
    using scalar_type = typename Inner::scalar_type;
    static constexpr int joints = Inner::joints;

    counting_chain(const Inner& inner, kernel_counts& counts)
        : m_inner(inner)
        , m_counts(counts)
    {
    }

    const Inner& inner() const { return m_inner; }

    /// A non-const reference out of a const accessor: the solver holds the
    /// adaptor as `const counting_chain&`, and the counters have to advance
    /// through that form or nothing is counted.
    kernel_counts& counts() const { return m_counts; }

    int num_joints() const { return m_inner.num_joints(); }

    const cartan::se3<scalar_type>& home() const { return m_inner.home(); }

    const typename Inner::screw_storage& axes() const { return m_inner.axes(); }

    const typename Inner::limits_storage& limits() const { return m_inner.limits(); }

    const cartan::screw_axis<scalar_type>& axis(int i) const { return m_inner.axis(i); }

private:
    const Inner& m_inner;
    kernel_counts& m_counts;
};

/// Delegation goes to the inner chain's own overload rather than to the generic
/// chain overload the adaptor would otherwise fall into: the generic one is the
/// quaternion path, so delegating there would change what is computed instead
/// of counting what happens.
template <typename Inner>
cartan::fk_result<typename Inner::scalar_type, Inner::joints> forward_kinematics_unchecked(
    const counting_chain<Inner>& chain,
    const typename cartan::joint_state<
        typename Inner::scalar_type, Inner::joints>::position_type& q)
{
    ++chain.counts().fk;
    return cartan::forward_kinematics_unchecked(chain.inner(), q);
}

template <typename Inner>
cartan::jacobian_matrix<typename Inner::scalar_type, Inner::joints> body_jacobian_unchecked(
    const counting_chain<Inner>& chain,
    const cartan::fk_result<typename Inner::scalar_type, Inner::joints>& fk)
{
    ++chain.counts().jac;
    return cartan::body_jacobian_unchecked(chain.inner(), fk);
}

}

#endif
