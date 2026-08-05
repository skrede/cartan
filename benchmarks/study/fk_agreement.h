#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_FK_AGREEMENT_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_FK_AGREEMENT_H

/// @file fk_agreement.h
/// @brief How far a description-loaded chain is from the robot it claims to be.
///
/// A description for the wrong variant of a robot is otherwise indistinguishable
/// from the right one, so each loaded chain is measured against the hand-coded
/// chain the previous study used. The gate refuses on it and the study's manifest
/// publishes it: the same measurement is the evidence in both places, so a
/// published capture cannot be sound about a robot the gate refuses.

#include "../../tests/fixtures/chain_factories.h"

#include <cartan/lie/se3.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <random>
#include <string>
#include <cstdint>
#include <numbers>
#include <utility>
#include <algorithm>
#include <stdexcept>
#include <string_view>

namespace cartan::bench
{

using truth_chain = cartan::kinematic_chain<double, cartan::dynamic>;
using fk_deviation = std::pair<double, double>;

struct description_truth
{
    std::string_view robot_key;
    truth_chain chain;
    std::uint64_t seed;
};

inline const std::vector<description_truth>& description_truths()
{
    using namespace cartan::fixtures;
    static const std::vector<description_truth> truths{
        {"abb_irb120", make_abb_irb120_chain_extended<double>(), 301ULL},
        {"kuka_kr6_r900", make_kr6_sixx_chain_extended<double>(), 300ULL},
        {"kuka_lbr_med14", make_iiwa14_chain_extended<double>(), 104ULL},
        {"franka_panda", make_panda_chain_extended<double>(), 200ULL},
        {"universal_robots_ur3e", make_ur3e_chain_extended<double>(), 100ULL}};
    return truths;
}

inline const description_truth& truth_for(std::string_view robot_key)
{
    for (const auto& truth : description_truths())
    {
        if (truth.robot_key == robot_key)
        {
            return truth;
        }
    }
    throw std::runtime_error(
        std::string{robot_key} + ": no hand-coded chain is registered to check it against");
}

namespace detail
{

inline fk_deviation pose_deviation(const cartan::se3<double>& a, const cartan::se3<double>& b)
{
    const auto twist = (a.inverse() * b).log();
    return {twist.tail<3>().norm(), twist.head<3>().norm()};
}

/// A continuous joint carries an infinite bound, which no uniform draw is
/// defined over. Agreement between two chains is a property of the pose map and
/// does not depend on sampling the whole line, so the draw is taken over the
/// bound's intersection with one turn.
template <int N>
Eigen::Matrix<double, N, 1> sample(
    const cartan::kinematic_chain<double, N>& chain, std::mt19937& rng)
{
    Eigen::Matrix<double, N, 1> q;
    for (int i = 0; i < N; ++i)
    {
        const auto& bound = chain.limits()[static_cast<std::size_t>(i)];
        std::uniform_real_distribution<double> draw(
            std::max(bound.position_min(), -std::numbers::pi),
            std::min(bound.position_max(), std::numbers::pi));
        q(i) = draw(rng);
    }
    return q;
}

}

constexpr int fk_agreement_configurations = 128;

template <int N>
fk_deviation worst_fk_deviation(
    const cartan::kinematic_chain<double, N>& loaded,
    const truth_chain& truth,
    std::uint64_t seed)
{
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    fk_deviation worst{0.0, 0.0};
    for (int i = 0; i < fk_agreement_configurations; ++i)
    {
        const Eigen::Matrix<double, N, 1> q = detail::sample(loaded, rng);
        const auto here = detail::pose_deviation(
            cartan::forward_kinematics(loaded, q).value().end_effector,
            cartan::forward_kinematics(truth, Eigen::VectorXd(q)).value().end_effector);
        worst = {std::max(worst.first, here.first), std::max(worst.second, here.second)};
    }
    return worst;
}

}

#endif
