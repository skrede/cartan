#include "../support/counting_chain.h"
#include "../support/kinematics_helpers.h"
#include "../support/joint_limits_helpers.h"

#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/chain_concept.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <numbers>

namespace
{

using chain_t = cartan::kinematic_chain<double, 6>;
using counted_t = cartan::testing::counting_chain<chain_t>;
using position_t = Eigen::Vector<double, 6>;

static_assert(cartan::chain<counted_t>);

/// The charges a completed drive is entitled to, and the exit it reaches. The
/// two are recorded together because the same charge count means one thing on
/// an iteration-limit exit and another on a converged one.
struct accounting_row
{
    int budget;
    std::int64_t fk;
    std::int64_t jac;
    bool converged;
};

struct solve_record
{
    position_t q;
    int iterations;
    bool converged;
};

chain_t make_ur5_like_chain()
{
    auto s1 = cartan::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = cartan::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    cartan::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    auto lim = cartan::testing::limits(-2 * std::numbers::pi, 2 * std::numbers::pi);
    return chain_t(home, {s1, s2, s3, s4, s5, s6}, {lim, lim, lim, lim, lim, lim});
}

cartan::convergence_criteria<double> criteria_at(int budget)
{
    cartan::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-5;
    criteria.orientation_tol = 1e-5;
    criteria.max_iterations_per_attempt = budget;
    criteria.max_total_work_units = budget;
    return criteria;
}

template <typename Chain>
solve_record drive(const Chain& chain, const cartan::se3<double>& target, int budget)
{
    cartan::basic_ik_runner<cartan::lm<Chain>> runner;
    const position_t q0 = position_t::Zero();

    runner.setup(chain, target, q0, criteria_at(budget));
    runner.solve();

    return solve_record{runner.current_q(), runner.iterations(), runner.converged()};
}

}

TEST_CASE("the LM loop's kernel charges per budget", "[ik][lm][accounting]")
{
    const chain_t chain = make_ur5_like_chain();

    position_t q_truth;
    q_truth << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    const cartan::se3<double> target = cartan::testing::fk_at(chain, q_truth).end_effector;

    const std::array<accounting_row, 4> table{{
        {3, 6, 3, false},
        {9, 12, 6, true},
        {27, 12, 6, true},
        {81, 12, 6, true},
    }};

    for (const accounting_row& row : table)
    {
        CAPTURE(row.budget);

        cartan::testing::kernel_counts counts{0, 0};
        const counted_t counted(chain, counts);

        const solve_record through = drive(counted, target, row.budget);
        const solve_record bare = drive(chain, target, row.budget);

        REQUIRE(counts.fk == row.fk);
        REQUIRE(counts.jac == row.jac);
        REQUIRE(counts.fk == 2 * counts.jac);

        REQUIRE(through.converged == row.converged);
        REQUIRE(through.iterations == (row.converged ? row.jac - 1 : row.jac));

        // What makes the counts attributable: substituting the adaptor has to
        // count the solve rather than compute a different one.
        for (int i = 0; i < 6; ++i)
        {
            CAPTURE(i);
            REQUIRE(through.q(i) == bare.q(i));
        }
    }
}
