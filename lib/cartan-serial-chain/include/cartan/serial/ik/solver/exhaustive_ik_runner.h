#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_EXHAUSTIVE_IK_RUNNER_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_EXHAUSTIVE_IK_RUNNER_H

/// @file exhaustive_ik_runner.h
/// @brief Exhaustive IK solver that collects all valid solutions via multi-start.

#include "cartan/serial/ik/ik_result.h"
#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/ik_validation.h"

#include "cartan/serial/ik/concepts/solve_concept.h"

#include "cartan/serial/ik/solver/detail/halton_seed_generator.h"

#include "cartan/lie/se3.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"

#include <algorithm>
#include <vector>

namespace cartan
{

enum class ranking_strategy
{
    distance_to_seed,
    min_error,
    mid_range
};

template <typename Scalar = double>
struct exhaustive_options
{
    int max_restarts{100};
    Scalar dedup_tolerance{Scalar(1e-3)};
    ranking_strategy ranking{ranking_strategy::distance_to_seed};
};

template <typename Scalar = double, int N = dynamic>
struct exhaustive_result
{
    std::vector<ik_result<Scalar, N>> solutions;
    int restarts_attempted{};
    int solutions_before_dedup{};
    int fk_validations_failed{};
};

template <chain Chain, typename Policy>
    requires ik::solve_policy<Policy>
class exhaustive_ik_runner
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>,
        "exhaustive_ik_runner requires a floating-point Scalar type");

    [[nodiscard]] exhaustive_result<scalar_type, joints> solve(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& seed,
        const convergence_criteria<scalar_type>& criteria,
        const exhaustive_options<scalar_type>& options = {})
    {
        exhaustive_result<scalar_type, joints> result{};
        halton_seed_generator<Chain> seed_gen{chain};
        std::vector<ik_result<scalar_type, joints>> validated;

        for (int restart = 0; restart < options.max_restarts; ++restart)
        {
            position_type seed_q = (restart == 0) ? seed : seed_gen(restart - 1);

            Policy policy{};
            policy.setup(chain, target, seed_q, criteria);

            for (int i = 0; i < criteria.max_iterations_per_attempt; ++i)
            {
                auto step_res = ik::step_one(policy, chain);
                if (step_res.status != ik_status::running)
                {
                    if (step_res.status == ik_status::converged)
                    {
                        auto q = policy.solution();
                        if (verify_solution(chain, target, q, criteria))
                        {
                            validated.push_back(ik_result<scalar_type, joints>{
                                .solution = joint_state<scalar_type, joints>::from_position(q),
                                .final_error_norm = policy.error_norm(),
                                .iterations = policy.iterations(),
                                .solver_index = restart
                            });
                        }
                        else
                        {
                            ++result.fk_validations_failed;
                        }
                    }
                    break;
                }
            }
        }

        result.restarts_attempted = options.max_restarts;
        result.solutions_before_dedup = static_cast<int>(validated.size());

        // O(n^2) dedup: keep first-encountered per cluster
        for (auto& candidate : validated)
        {
            bool is_dup = false;
            for (const auto& kept : result.solutions)
            {
                if ((candidate.solution.position - kept.solution.position).norm()
                    < options.dedup_tolerance)
                {
                    is_dup = true;
                    break;
                }
            }
            if (!is_dup)
                result.solutions.push_back(std::move(candidate));
        }

        rank(result.solutions, seed, chain, options.ranking);
        return result;
    }

private:
    static void rank(
        std::vector<ik_result<scalar_type, joints>>& solutions,
        const position_type& seed,
        const Chain& chain,
        ranking_strategy strategy)
    {
        if (solutions.size() < 2)
            return;

        switch (strategy)
        {
            case ranking_strategy::distance_to_seed:
                std::ranges::sort(solutions, [&](const auto& a, const auto& b)
                {
                    return (a.solution.position - seed).norm()
                         < (b.solution.position - seed).norm();
                });
                break;

            case ranking_strategy::min_error:
                std::ranges::sort(solutions, [](const auto& a, const auto& b)
                {
                    return a.final_error_norm < b.final_error_norm;
                });
                break;

            case ranking_strategy::mid_range:
            {
                position_type mid;
                int n = chain.num_joints();
                if constexpr (joints == dynamic)
                    mid.resize(n);
                for (int j = 0; j < n; ++j)
                {
                    auto lim = chain.limits()[static_cast<std::size_t>(j)];
                    mid[j] = (lim.position_min + lim.position_max) / scalar_type(2);
                }
                std::ranges::sort(solutions, [&](const auto& a, const auto& b)
                {
                    return (a.solution.position - mid).norm()
                         < (b.solution.position - mid).norm();
                });
                break;
            }
        }
    }
};

}

#endif
