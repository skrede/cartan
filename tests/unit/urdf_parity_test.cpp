#include "../fixtures/chain_factories.h"

#include <cartan/urdf.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <random>
#include <string>
#include <cstdint>
#include <filesystem>

/// Parity tests asserting that the URDF loader and the hand-coded ground-truth
/// kinematic_chain factories agree on forward kinematics to within 1e-12 across
/// 100 random reachable joint configurations. The synthetic cartanbot fixture
/// is always exercised; the vendored real-world fixtures (UR3e / UR5e / UR10 /
/// UR16 / iiwa14) compile in only under CARTAN_URDF_EXTENDED_TESTS.

namespace
{

namespace fs = std::filesystem;

template <typename Scalar>
[[nodiscard]] auto random_within_limits(
    const cartan::kinematic_chain<Scalar, cartan::dynamic>& chain,
    std::mt19937& rng) -> Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
{
    const int n = chain.num_joints();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q(n);
    for (int i = 0; i < n; ++i)
    {
        const auto& lim = chain.limits()[static_cast<std::size_t>(i)];
        Scalar lo = lim.position_min;
        Scalar hi = lim.position_max;
        // Clamp infinities to a reasonable bound (continuous joints) so the
        // uniform sample is well-defined; the parity test does not depend on
        // sampling the whole real line.
        if (!std::isfinite(lo)) lo = -Scalar(3.14159265358979);
        if (!std::isfinite(hi)) hi = +Scalar(3.14159265358979);
        std::uniform_real_distribution<Scalar> dist(lo, hi);
        q(i) = dist(rng);
    }
    return q;
}

template <typename Scalar>
[[nodiscard]] auto pose_error_norm(
    const cartan::se3<Scalar>& a,
    const cartan::se3<Scalar>& b) -> std::pair<Scalar, Scalar>
{
    auto err_twist = (a.inverse() * b).log();
    Scalar omega = err_twist.template head<3>().norm();
    Scalar v = err_twist.template tail<3>().norm();
    return {v, omega};
}

template <typename TruthChain>
void check_parity(
    const fs::path& urdf_path,
    const TruthChain& truth,
    std::uint64_t seed)
{
    using Scalar = double;
    auto loaded = cartan::load_urdf<Scalar>(urdf_path);
    REQUIRE(loaded.has_value());
    const auto& loaded_chain = loaded->chain;

    REQUIRE(loaded_chain.num_joints() == truth.num_joints());

    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    for (int i = 0; i < 100; ++i)
    {
        auto q = random_within_limits(truth, rng);
        auto fk_truth = cartan::forward_kinematics(truth, q);
        auto fk_loaded = cartan::forward_kinematics(loaded_chain, q);

        auto [pos_err, ori_err] = pose_error_norm(fk_loaded.end_effector, fk_truth.end_effector);
        REQUIRE(pos_err < Scalar(1e-12));
        REQUIRE(ori_err < Scalar(1e-12));
    }
}

}

TEST_CASE("urdf parity: cartanbot URDF matches make_cartanbot_chain at 1e-12", "[urdf_parity]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "cartanbot.urdf";
    auto truth = cartan::fixtures::make_cartanbot_chain<double>();
    check_parity(urdf_path, truth, 42ULL);
}

#ifdef CARTAN_URDF_EXTENDED_TESTS

TEST_CASE("urdf parity: UR3e URDF matches make_ur3e_chain_extended at 1e-12", "[urdf_parity][extended]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "extended" / "ur3e.urdf";
    auto truth = cartan::fixtures::make_ur3e_chain_extended<double>();
    check_parity(urdf_path, truth, 100ULL);
}

TEST_CASE("urdf parity: UR5e URDF matches make_ur5e_chain_extended at 1e-12", "[urdf_parity][extended]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "extended" / "ur5e.urdf";
    auto truth = cartan::fixtures::make_ur5e_chain_extended<double>();
    check_parity(urdf_path, truth, 101ULL);
}

TEST_CASE("urdf parity: UR10 URDF matches make_ur10_chain_extended at 1e-12", "[urdf_parity][extended]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "extended" / "ur10.urdf";
    auto truth = cartan::fixtures::make_ur10_chain_extended<double>();
    check_parity(urdf_path, truth, 102ULL);
}

TEST_CASE("urdf parity: UR16 URDF matches make_ur16_chain_extended at 1e-12", "[urdf_parity][extended]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "extended" / "ur16.urdf";
    auto truth = cartan::fixtures::make_ur16_chain_extended<double>();
    check_parity(urdf_path, truth, 103ULL);
}

TEST_CASE("urdf parity: iiwa14 URDF matches make_iiwa14_chain_extended at 1e-12", "[urdf_parity][extended]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "extended" / "iiwa14.urdf";
    auto truth = cartan::fixtures::make_iiwa14_chain_extended<double>();
    check_parity(urdf_path, truth, 104ULL);
}

TEST_CASE("urdf parity: Panda URDF matches make_panda_chain_extended at 1e-12", "[urdf_parity][extended]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "extended" / "panda.urdf";
    auto truth = cartan::fixtures::make_panda_chain_extended<double>();
    check_parity(urdf_path, truth, 200ULL);
}

TEST_CASE("urdf parity: iiwa7 URDF matches make_lbr_iiwa7_chain_extended at 1e-12", "[urdf_parity][extended]")
{
    const fs::path urdf_path = fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "extended" / "iiwa7.urdf";
    auto truth = cartan::fixtures::make_lbr_iiwa7_chain_extended<double>();
    check_parity(urdf_path, truth, 201ULL);
}

#endif
