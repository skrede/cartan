/// @file fk_sweep_test.cpp
/// @brief Seeded-random forward-kinematics regression sweep.
///
/// For each of the nine benchmark robots the fixed-size PoE forward
/// kinematics is checked over fifty seeded random configurations, drawn
/// uniformly within the joint limits, against two independent oracles:
///   1. an explicit se3::exp Product-of-Exponentials evaluation, built with a
///      hand loop so it does not share the forward_kinematics dispatch under
///      test, and
///   2. the dynamic-size chain evaluated on the same configuration, guarding
///      the fixed-size fast path against the runtime path.
/// Both must agree with the fixed-size result to 1e-10, so a regression in the
/// PoE formula or in either specialization fails the sweep on the offending
/// robot and configuration. Coverage extends past the revolute-only robots to
/// a pure-prismatic chain, a mixed revolute/prismatic chain, and a zero-DOF
/// chain.
///
/// The pattern (mt19937 seed 42, per-joint uniform-in-limits draw, 1e-10 gate)
/// mirrors the KDL comparison sweep; the deterministic seed makes any failure
/// reproducible.

#include "../fixtures/chain_factories.h"
#include "../fixtures/prismatic_chains.h"

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <random>
#include <vector>
#include <cstddef>

namespace
{

constexpr double sweep_tol = 1e-10;
constexpr int sweep_trials = 50;

/// Independent space-form Product-of-Exponentials oracle:
///   T(q) = exp([S1] q1) ... exp([Sn] qn) * M
/// The explicit se3::exp loop shares no code with the forward_kinematics
/// dispatch under test, so agreement is a genuine correctness signal.
template <typename Chain, typename Vec>
auto poe_oracle(const Chain& chain, const Vec& q)
    -> cartan::se3<typename Chain::scalar_type>
{
    using Scalar = typename Chain::scalar_type;
    auto T = cartan::se3<Scalar>::identity();
    const int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        const cartan::vector6<Scalar> screw = chain.axis(i).to_vector();
        T = T * cartan::se3<Scalar>::exp((screw * q(i)).eval());
    }
    return T * chain.home();
}

template <typename Scalar>
Scalar pose_error(const cartan::se3<Scalar>& a, const cartan::se3<Scalar>& b)
{
    return (a.inverse() * b).log().norm();
}

/// One robot: fifty seeded random configs, fixed FK vs PoE oracle and vs the
/// dynamic-size path.
template <typename MakeChain>
void fk_sweep_robot(MakeChain make_chain, const char* name)
{
    INFO("Robot: " << name);

    auto chain = make_chain();
    using Chain = decltype(chain);
    using Scalar = typename Chain::scalar_type;
    constexpr int N = Chain::joints;

    auto dyn = chain.to_dynamic();
    const int n = chain.num_joints();

    std::mt19937 rng(42);
    std::uniform_real_distribution<Scalar> unit(Scalar(0), Scalar(1));

    for (int trial = 0; trial < sweep_trials; ++trial)
    {
        INFO("Trial: " << trial);

        Eigen::Vector<Scalar, N> q;
        Eigen::VectorX<Scalar> q_dyn(n);
        for (int j = 0; j < n; ++j)
        {
            const auto& lim = chain.limits()[static_cast<std::size_t>(j)];
            q(j) = lim.position_min
                 + (lim.position_max - lim.position_min) * unit(rng);
            q_dyn(j) = q(j);
        }

        auto fk = cartan::forward_kinematics(chain, q).end_effector;
        auto oracle = poe_oracle(chain, q);
        auto fk_dyn = cartan::forward_kinematics(dyn, q_dyn).end_effector;

        REQUIRE(pose_error(fk, oracle) < Scalar(sweep_tol));
        REQUIRE(pose_error(fk, fk_dyn) < Scalar(sweep_tol));
    }
}

/// Pure-prismatic P-P-P gantry (signed directions +z, +x, -y). The negative
/// third direction exercises the prismatic sign path against the PoE oracle.
template <typename Scalar>
auto make_ppp_chain() -> cartan::kinematic_chain<Scalar, 3>
{
    using vec3 = cartan::vector3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(0), Scalar(0), Scalar(1)));
    auto s2 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(1), Scalar(0), Scalar(0)));
    auto s3 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(0), Scalar(-1), Scalar(0)));

    auto home = cartan::se3<Scalar>(
        cartan::so3<Scalar>::identity(), vec3(Scalar(0), Scalar(0), Scalar(0)));
    cartan::joint_limits<Scalar> lim{Scalar(-1), Scalar(1)};

    return cartan::kinematic_chain<Scalar, 3>(
        home, {s1, s2, s3}, {lim, lim, lim});
}

/// Zero-DOF dynamic chain: empty axis/limit storage, non-trivial home pose.
template <typename Scalar>
auto make_zero_dof_chain() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    auto home = cartan::se3<Scalar>(
        cartan::so3<Scalar>::identity(),
        cartan::vector3<Scalar>(Scalar(0.1), Scalar(0.2), Scalar(0.3)));
    return cartan::kinematic_chain<Scalar, cartan::dynamic>(
        home,
        std::vector<cartan::screw_axis<Scalar>>{},
        std::vector<cartan::joint_limits<Scalar>>{});
}

}

TEST_CASE("FK sweep: nine robots x fifty seeded configs", "[fk][sweep]")
{
    fk_sweep_robot(cartan::fixtures::make_ur3e_chain<double>, "UR3e");
    fk_sweep_robot(cartan::fixtures::make_lbr_med14_chain<double>, "LBR Med 14");
    fk_sweep_robot(cartan::fixtures::make_kr6_sixx_chain<double>, "KR6 SIXX");
    fk_sweep_robot(cartan::fixtures::make_panda_chain<double>, "Panda");
    fk_sweep_robot(cartan::fixtures::make_abb_irb120_chain<double>, "ABB IRB 120");
    fk_sweep_robot(cartan::fixtures::make_jaco2_chain<double>, "Jaco2");
    fk_sweep_robot(cartan::fixtures::make_fetch_chain<double>, "Fetch");
    fk_sweep_robot(cartan::fixtures::make_baxter_chain<double>, "Baxter");
    fk_sweep_robot(cartan::fixtures::make_kuka_lwr4_chain<double>, "Kuka LWR4");
}

TEST_CASE("FK sweep: prismatic, mixed, and zero-DOF coverage",
    "[fk][sweep][prismatic]")
{
    SECTION("pure prismatic chain")
    {
        fk_sweep_robot(make_ppp_chain<double>, "PPP prismatic");
    }

    SECTION("mixed revolute/prismatic chain")
    {
        fk_sweep_robot(
            cartan::fixtures::make_rppr_signed_chain<double>, "RPPR mixed");
    }

    SECTION("zero-DOF chain returns the home pose")
    {
        auto chain = make_zero_dof_chain<double>();
        REQUIRE(chain.num_joints() == 0);

        Eigen::VectorX<double> q(0);
        auto fk = cartan::forward_kinematics(chain, q).end_effector;
        REQUIRE(pose_error(fk, chain.home()) < sweep_tol);
    }
}
