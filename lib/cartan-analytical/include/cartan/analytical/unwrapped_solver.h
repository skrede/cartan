#ifndef HPP_GUARD_CARTAN_ANALYTICAL_UNWRAPPED_SOLVER_H
#define HPP_GUARD_CARTAN_ANALYTICAL_UNWRAPPED_SOLVER_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/unwrapped_result.h"
#include "cartan/analytical/detail/angle_unwrap.h"

#include "cartan/serial/ik/detail/limit_enforcement.h"

#include "cartan/lie/se3.h"

#include "cartan/expected.h"

#include <utility>

namespace cartan
{

/// Composable post-filter over any analytical solver: it 2*pi-unwraps every
/// returned branch per zero-pitch-revolute joint into the chain's declared
/// range, tags each branch in_range / joint_limits_violated, and returns them
/// all in an unwrapped_result -- leaving the inner analytical_result untouched.
/// solve(target) unwraps toward zero; solve(target, q_seed) toward q_seed, so
/// one instance tracks a trajectory by passing the evolving pose each call. A
/// whole-solve failure of the inner solver is passed through unchanged. This is
/// a sibling type, NOT a model of the analytical_solver concept (whose same_as
/// return-type constraint forbids a new result type); it mirrors restart_wrapper
/// only in ergonomics -- single-arg construction plus a CTAD deduction guide.
template <typename Inner>
class unwrapped_solver
{
    using reference_type = Eigen::Vector<typename Inner::scalar_type, Inner::joints>;

public:
    using inner_type = Inner;
    using chain_type = typename Inner::chain_type;
    using scalar_type = typename Inner::scalar_type;
    static constexpr int joints = Inner::joints;
    static constexpr int max_solutions = Inner::max_solutions;
    using result_type = unwrapped_result<scalar_type, joints, max_solutions>;

    explicit unwrapped_solver(Inner inner)
        : m_inner(std::move(inner))
    {
    }

    cartan::expected<result_type, analytical_error<scalar_type>>
    solve(const se3<scalar_type>& target) const
    {
        return solve_impl(target, reference_type::Zero());
    }

    cartan::expected<result_type, analytical_error<scalar_type>>
    solve(const se3<scalar_type>& target, const reference_type& q_seed) const
    {
        return solve_impl(target, q_seed);
    }

private:
    cartan::expected<result_type, analytical_error<scalar_type>>
    solve_impl(const se3<scalar_type>& target, const reference_type& reference) const
    {
        auto inner = m_inner.solve(target);
        if (!inner)
        {
            return cartan::unexpected(inner.error());
        }
        const chain_type& chain = m_inner.chain();
        const scalar_type tol = cartan::detail::default_feasibility_tol<scalar_type>();
        result_type out;
        out.count = inner->count;
        for (int b = 0; b < inner->count; ++b)
        {
            auto idx = static_cast<std::size_t>(b);
            out.solutions[idx] =
                unwrap_branch(inner->solutions[idx], chain, reference, tol);
            out.tags[idx] = cartan::detail::within_limits(out.solutions[idx], chain, tol)
                ? range_status::in_range
                : range_status::joint_limits_violated;
        }
        return out;
    }

    static reference_type unwrap_branch(
        const reference_type& q, const chain_type& chain,
        const reference_type& reference, scalar_type tol)
    {
        reference_type out;
        const auto& limits = chain.limits();
        for (int i = 0; i < joints; ++i)
        {
            auto idx = static_cast<std::size_t>(i);
            if (!cartan::detail::is_zero_pitch_revolute(chain.axis(i)))
            {
                out(i) = q(i);
                continue;
            }
            out(i) = cartan::detail::unwrap_to_range_nearest(
                q(i), limits[idx].position_min, limits[idx].position_max,
                reference(i), tol);
        }
        return out;
    }

    Inner m_inner;
};

template <typename Inner>
unwrapped_solver(Inner) -> unwrapped_solver<Inner>;

}

#endif
