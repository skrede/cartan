#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/opw_chains.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <random>
#include <numbers>

using namespace cartan;

namespace
{

/// Position and orientation disagreement between the closed-form OPW forward map
/// and the independent screw-model forward kinematics at a single configuration.
struct reconstruction_error
{
    double position;     ///< || t_opw - t_screw || in meters.
    double orientation;  ///< || log(R_opw^-1 * R_screw) || in radians.
};

reconstruction_error reconstruction_at(
    const opw_parameters<double>& params,
    const Eigen::Vector<double, 6>& q,
    const se3<double>& screw_pose)
{
    const se3<double> opw_pose = opw_forward(params, q);

    const double position =
        (opw_pose.translation() - screw_pose.translation()).norm();
    const double orientation =
        (opw_pose.rotation().inverse() * screw_pose.rotation()).log().norm();

    return {position, orientation};
}

}

/// The hand-derived KR6 R900 OPW parameters must reconstruct the authoritative
/// screw model exactly. Reproducing the OPW forward map from the parameters and
/// comparing against the chain's own product-of-exponentials forward kinematics
/// at more than a thousand configurations pins every length, offset and sign at
/// 1e-9. A wrong assumption (for example an incorrect elbow offset a2, a swapped
/// sign, or an R700 upper-arm length) surfaces here as a mismatch far above the
/// tolerance, rather than a self-consistent but wrong robot slipping through.
TEST_CASE(
    "OPW forward map reconstructs the KR6 R900 screw model at 1e-9",
    "[analytical][opw]")
{
    const auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    const auto params = fixtures::kr6_r900_opw_parameters<double>();

    constexpr double tolerance = 1e-9;
    const double pi = std::numbers::pi_v<double>;

    // The home configuration is the single most convention-sensitive pose (it
    // exercises offsets[1] = -pi/2 and every sign flip at once); check it first
    // so a convention slip is diagnosed before the random sweep.
    {
        const Eigen::Vector<double, 6> q = Eigen::Vector<double, 6>::Zero();
        const auto fk = forward_kinematics(chain, q).end_effector;
        const auto err = reconstruction_at(params, q, fk);
        INFO("home configuration q = " << q.transpose());
        CHECK(err.position < tolerance);
        CHECK(err.orientation < tolerance);
    }

    // Deterministic full-range sweep: a fixed-seed PRNG draws each joint from the
    // whole [-pi, pi] limit so a failing configuration is reproducible.
    std::mt19937_64 rng(0xC0FFEEULL);
    std::uniform_real_distribution<double> joint(-pi, pi);

    constexpr int sample_count = 2000;
    double worst_position = 0.0;
    double worst_orientation = 0.0;

    for (int sample = 0; sample < sample_count; ++sample)
    {
        Eigen::Vector<double, 6> q;
        for (int j = 0; j < 6; ++j)
            q(j) = joint(rng);

        const auto fk = forward_kinematics(chain, q).end_effector;
        const auto err = reconstruction_at(params, q, fk);

        worst_position = std::max(worst_position, err.position);
        worst_orientation = std::max(worst_orientation, err.orientation);

        INFO("sample " << sample << " q = " << q.transpose());
        INFO("position error " << err.position
                               << " orientation error " << err.orientation);
        REQUIRE(err.position < tolerance);
        REQUIRE(err.orientation < tolerance);
    }

    INFO("worst position error " << worst_position
                                 << " worst orientation error "
                                 << worst_orientation);
    CHECK(worst_position < tolerance);
    CHECK(worst_orientation < tolerance);
}
