// The steady-state zero-allocation proof for projected_lm.
//
// The include ordering below is load-bearing and deliberately deviates from the
// usual project ordering: Eigen's runtime no-malloc trap and a throwing
// eigen_assert MUST be established before the first Eigen header is pulled in
// (directly or transitively through a cartan header). Redefining eigen_assert to
// throw is what makes the trap survive -DNDEBUG, where Eigen's default
// assert-based trap is compiled out. EIGEN_RUNTIME_NO_MALLOC then gates every
// Eigen allocation on set_is_malloc_allowed().
#include <stdexcept>

#define EIGEN_RUNTIME_NO_MALLOC
#define eigen_assert(X) do { if (!(X)) throw std::runtime_error("eigen-malloc"); } while (0)
#include <Eigen/Dense>

#include "../fixtures/chain_factories.h"
#include "../support/alloc_counter.h"

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/ik/solver/projected_lm.h>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <random>

// ============================================================================
// Zero-allocation guarantee for projected_lm's steady-state step loop.
//
// projected_lm solves the per-step active-set system with an LDLT factorization
// over max-size-fixed temporaries, so on a fixed-N chain it must perform no heap
// allocation once its setup has run. This test proves that with two independent
// mechanisms armed around the steady-state descent:
//
//   1. Eigen's runtime no-malloc trap (set_is_malloc_allowed(false)), made
//      NDEBUG-effective by the throwing eigen_assert defined above. This catches
//      Eigen's aligned allocator, which reaches std::malloc directly and would
//      slip past an operator-new counter.
//   2. A global operator new/delete counter (alloc_counter.h). This catches any
//      non-Eigen heap allocation, which the Eigen trap does not observe.
//
// Both are needed to be airtight, and the process-global nature of the malloc
// flag and the counter is why this test is registered RUN_SERIAL. All setup
// allocation (chain construction, solver setup, a warm-up step) happens before
// the mechanisms are armed; only the steady-state loop runs inside the armed
// region. The guarantee is scoped to projected_lm (LDLT); dls uses a JacobiSVD
// that may allocate internally, so no dls zero-alloc assertion is made here.
// ============================================================================

namespace
{

template <typename Scalar>
void run_zero_alloc_case()
{
    auto chain = cartan::fixtures::make_ur3e_chain<Scalar>();
    using chain_type = cartan::kinematic_chain<Scalar, 6>;
    using position_type = typename cartan::joint_state<Scalar, 6>::position_type;

    std::mt19937 rng(42);
    const auto target = cartan::fixtures::random_reachable_target(chain, rng);

    cartan::convergence_criteria<Scalar> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

    cartan::projected_lm<chain_type> solver;
    const position_type q0 = position_type::Zero();

    // All setup allocation happens here, before the trap and counter are armed.
    solver.setup(chain, target, q0, criteria);

    // A warm-up step flushes any first-iteration lazy allocation so the armed
    // region observes only steady-state descent.
    solver.step(chain, 1);

    // Arm both mechanisms. From here, any heap allocation either trips the Eigen
    // trap (throwing out of step) or increments the operator-new counter.
    Eigen::internal::set_is_malloc_allowed(false);
    const std::size_t before = cartan::testing::alloc_count();

    int steps = 0;
    for (; steps < 400 && !solver.converged(); ++steps)
    {
        solver.step(chain, 1);
    }

    const std::size_t after = cartan::testing::alloc_count();
    Eigen::internal::set_is_malloc_allowed(true);

    // The steady-state loop allocated nothing on the operator-new path, and the
    // Eigen trap never fired (it would have thrown before reaching here).
    REQUIRE(after - before == 0);

    // The armed loop must actually have exercised real iterations and driven the
    // solve to completion, so the zero-alloc result has teeth rather than being a
    // no-op over an already-converged solver.
    REQUIRE(steps > 0);
    REQUIRE(solver.converged());
}

}

TEST_CASE("projected_lm steady-state step loop allocates no heap (double)", "[ik][projected_lm][alloc]")
{
    run_zero_alloc_case<double>();
}

TEST_CASE("projected_lm steady-state step loop allocates no heap (float)", "[ik][projected_lm][alloc]")
{
    run_zero_alloc_case<float>();
}
