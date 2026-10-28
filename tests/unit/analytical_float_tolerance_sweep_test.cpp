// Float acceptance tolerance for the analytical solvers: the measurement behind
// the specialized default, and a regression gate on it.
//
// The module shipped one verification tolerance for every scalar, 1e-6 in both
// fields. That is about eight float epsilons -- below what a float forward map
// reconstructs on a metre-scale arm -- and the field is read both by the
// forward-kinematics back-check and by the Paden-Kahan subproblem gates the
// Pieper decomposition runs before it. A target the mechanism demonstrably
// reaches is therefore reported as a whole-solve failure, which is the worst
// shape a numerical defect takes: the caller is told there is no answer.
//
// RECORDED SWEEP (1024 reachable poses per solver, joint values drawn uniformly
// over (-pi, pi] from a fixed generator; the same poses at both scalars):
//
//   solver / scalar       tolerance | whole-solve failures | branches per pose
//   ortho-parallel double   1e-6    |    0 / 1024          | 7.67
//   ortho-parallel float    1e-6    |   51 / 1024          | 5.77
//   ortho-parallel double   1e-4    |    0 / 1024          | 7.67
//   ortho-parallel float    1e-4    |    0 / 1024          | 7.66
//   ortho-parallel raw float 1e-6   |    0 / 1024          | 8.00
//   Pieper double           1e-6    |    0 / 1024          | 8.00
//   Pieper float            1e-6    |   23 / 1024          | 6.42
//   Pieper double           1e-4    |    1 / 1024          | 7.99
//   Pieper float            1e-4    |    1 / 1024          | 7.99
//
// Two readings follow. The variant with the back-check removed answers all 1024
// poses at float with all eight branches, so the arithmetic reaches every branch
// and the loss is at the gate. And at 1e-4 the float sweep refuses exactly the
// pose the double sweep refuses at the same value, so nothing float-specific is
// left: widening the field also widens the subproblem gates that share it, and
// at 1e-3 seven of the 1024 are refused at double as well. That is why the value
// is 1e-4 -- the smallest decade at which float loses nothing double keeps.
//
// The live checks recompute every figure, so the table is documentation rather
// than the source of truth.

#include "../support/kinematics_helpers.h"

#include "../fixtures/opw_chains.h"
#include "../fixtures/chain_factories.h"

#include <cartan/analytical.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <vector>
#include <random>
#include <cstdint>
#include <numbers>

namespace spp = cartan;

namespace
{

constexpr int sweep_poses = 1024;

template <typename Scalar>
spp::verification_tolerance<Scalar> shipped_tolerance()
{
    return spp::verification_tolerance<Scalar>(Scalar(1e-6), Scalar(1e-6));
}

struct sweep_result
{
    std::vector<int> refused;
    double branches_per_pose{0.0};
};

/// Draws the same joint values at either scalar, so a refusal at float that is
/// absent at double is a property of the scalar and not of the pose.
template <typename Scalar, typename Chain, typename Solver>
sweep_result run_sweep(const Chain& chain, const Solver& solver, std::uint64_t seed)
{
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> angle(
        -std::numbers::pi, std::numbers::pi);

    sweep_result out;
    int branches = 0;
    for (int p = 0; p < sweep_poses; ++p)
    {
        Eigen::Vector<Scalar, 6> q;
        for (int k = 0; k < 6; ++k)
            q(k) = static_cast<Scalar>(angle(rng));

        auto solved = solver.solve(spp::testing::fk_at(chain, q).end_effector);
        if (solved)
            branches += solved->count;
        else
            out.refused.push_back(p);
    }
    out.branches_per_pose = double(branches) / double(sweep_poses);
    return out;
}

sweep_result report(const char* label, sweep_result swept)
{
    std::printf("[analytical-float-sweep] %-32s refused %4zu / %d, %.3f branches per pose\n",
        label, swept.refused.size(), sweep_poses, swept.branches_per_pose);
    return swept;
}

/// The tolerance defaults to the module constant, which is the argument make()
/// itself defaults to, so an unargumented call sweeps the shipping behavior.
template <typename Scalar, typename Verification = spp::opw_verified>
sweep_result opw_sweep(const char* label,
    spp::verification_tolerance<Scalar> tolerance
        = spp::default_verification_tolerance_v<Scalar>)
{
    auto chain = spp::fixtures::make_kr6_r900_opw_chain<Scalar>();
    auto solver = spp::opw_6r_solver<decltype(chain), Verification>::make(
        chain, spp::fixtures::kr6_r900_opw_parameters<Scalar>(), tolerance);
    REQUIRE(solver.has_value());
    return report(label, run_sweep<Scalar>(chain, *solver, 0xC0FFEEull));
}

template <typename Scalar>
sweep_result pieper_sweep(const char* label,
    spp::verification_tolerance<Scalar> tolerance
        = spp::default_verification_tolerance_v<Scalar>)
{
    auto chain = spp::fixtures::make_abb_irb120_chain<Scalar>();
    auto solver = spp::pieper_6r_solver<decltype(chain)>::make(chain, tolerance);
    REQUIRE(solver.has_value());
    return report(label, run_sweep<Scalar>(chain, *solver, 0xABB120ull));
}

}

TEST_CASE("the analytical verification tolerance is per scalar",
    "[analytical][float][sweep]")
{
    CHECK(spp::default_verification_tolerance_v<double>.position() == 1e-6);
    CHECK(spp::default_verification_tolerance_v<double>.orientation() == 1e-6);
    CHECK(spp::default_verification_tolerance_v<float>.position() == 1e-4f);
    CHECK(spp::default_verification_tolerance_v<float>.orientation() == 1e-4f);

    using float_chain = decltype(spp::fixtures::make_kr6_r900_opw_chain<float>());
    using float_solver = spp::opw_6r_solver<float_chain>;
    CHECK(float_solver::default_position_tolerance == 1e-4f);
    CHECK(float_solver::default_orientation_tolerance == 1e-4f);
}

TEST_CASE("the ortho-parallel solver answers at float only above the shipped tolerance",
    "[analytical][float][sweep]")
{
    const sweep_result forced_double =
        opw_sweep<double>("opw double @ 1e-6", shipped_tolerance<double>());
    const sweep_result forced_float =
        opw_sweep<float>("opw float @ 1e-6", shipped_tolerance<float>());
    const sweep_result raw_float = opw_sweep<float, spp::opw_raw>(
        "opw unverified float @ 1e-6", shipped_tolerance<float>());
    const sweep_result default_double = opw_sweep<double>(
        "opw double @ 1e-4", spp::verification_tolerance<double>(1e-4, 1e-4));
    const sweep_result default_float = opw_sweep<float>("opw float @ default");

    CHECK(forced_double.refused.empty());
    CHECK(forced_float.refused.size() >= 20);
    CHECK(forced_float.branches_per_pose < 6.5);

    // The back-check removed, every branch survives -- so the arithmetic reaches
    // them and the shipped gate is what discards them.
    CHECK(raw_float.refused.empty());
    CHECK(raw_float.branches_per_pose == 8.0);

    CHECK(default_float.refused == default_double.refused);
    CHECK(default_float.refused.empty());
    CHECK(default_float.branches_per_pose > 7.5);
}

TEST_CASE("the Pieper solver answers at float only above the shipped tolerance",
    "[analytical][float][sweep]")
{
    const sweep_result forced_double =
        pieper_sweep<double>("Pieper double @ 1e-6", shipped_tolerance<double>());
    const sweep_result forced_float =
        pieper_sweep<float>("Pieper float @ 1e-6", shipped_tolerance<float>());
    const sweep_result default_double = pieper_sweep<double>(
        "Pieper double @ 1e-4", spp::verification_tolerance<double>(1e-4, 1e-4));
    const sweep_result default_float = pieper_sweep<float>("Pieper float @ default");

    CHECK(forced_double.refused.empty());
    CHECK(forced_float.refused.size() >= 10);
    CHECK(forced_float.branches_per_pose < 7.0);

    // Not "refuses nothing": the pose the wider gate turns away is refused at
    // double too, and equality is the statement that nothing float-specific is
    // left. A regression to the shipped value breaks it in one direction and a
    // solver that stopped answering at double in the other.
    CHECK(default_float.refused == default_double.refused);
    CHECK(default_float.refused.size() <= 2);
    CHECK(default_float.branches_per_pose > 7.9);
}
