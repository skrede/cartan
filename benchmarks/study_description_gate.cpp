/// @file study_description_gate.cpp
/// @brief Proves the description-loaded chains are the robots they replace.
///
/// The study's feasible set moved from a synthetic symmetric box to the bounds
/// an upstream description declares. A description for the wrong variant of a
/// robot is otherwise indistinguishable from the right one, so each loaded
/// chain is checked against the hand-coded chain the previous study measured by
/// forward-kinematics agreement, and the measured maximum is printed.

#include "study/description_chain.h"

#include "../tests/fixtures/chain_factories.h"

#include <cartan/serial/fk/forward_kinematics.h>

#include <Eigen/Dense>

#include <cmath>
#include <numbers>
#include <random>
#include <string>
#include <cstdio>
#include <cstdint>
#include <utility>
#include <algorithm>
#include <exception>

namespace
{

using dynamic_chain = cartan::kinematic_chain<double, cartan::dynamic>;
using deviation = std::pair<double, double>;

constexpr int configurations = 128;
constexpr double agreement = 1e-9;

deviation pose_deviation(const cartan::se3<double>& a, const cartan::se3<double>& b)
{
    const auto twist = (a.inverse() * b).log();
    return {twist.tail<3>().norm(), twist.head<3>().norm()};
}

// A continuous joint carries an infinite bound, which no uniform draw is
// defined over. Agreement between two chains is a property of the pose map and
// does not depend on sampling the whole line, so the draw is taken over the
// bound's intersection with one turn.
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

template <int N>
deviation worst_deviation(
    const cartan::kinematic_chain<double, N>& loaded,
    const dynamic_chain& truth,
    std::uint64_t seed)
{
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    deviation worst{0.0, 0.0};
    for (int i = 0; i < configurations; ++i)
    {
        const Eigen::Matrix<double, N, 1> q = sample(loaded, rng);
        const auto here = pose_deviation(
            cartan::forward_kinematics(loaded, q).value().end_effector,
            cartan::forward_kinematics(truth, Eigen::VectorXd(q)).value().end_effector);
        worst = {std::max(worst.first, here.first), std::max(worst.second, here.second)};
    }
    return worst;
}

template <int N>
void print_bounds(std::string_view robot_key, const cartan::kinematic_chain<double, N>& chain)
{
    for (int i = 0; i < N; ++i)
    {
        const auto& bound = chain.limits()[static_cast<std::size_t>(i)];
        std::printf("%s joint %d: [%+.6f, %+.6f] rad\n",
            std::string(robot_key).c_str(), i + 1, bound.position_min(), bound.position_max());
    }
}

// Each robot is reported whether or not an earlier one failed: a gate that
// stops at the first refusal says nothing about the four descriptions behind it.
template <int N>
bool agrees(
    const cartan::bench::description_spec& spec, const dynamic_chain& truth, std::uint64_t seed)
{
    try
    {
        const auto loaded = cartan::bench::chain_from_description<N>(spec);
        print_bounds(spec.robot_key, loaded);
        const auto [position, orientation] = worst_deviation(loaded, truth, seed);
        std::printf("%s: max FK deviation over %d configurations %.3e m, %.3e rad\n\n",
            std::string(spec.robot_key).c_str(), configurations, position, orientation);
        return position < agreement && orientation < agreement;
    }
    catch (const std::exception& refusal)
    {
        std::printf("%s\n\n", refusal.what());
        return false;
    }
}

// A refusal only demonstrated by hand is a refusal nobody reruns, so the
// joint-count disagreement is provoked here rather than described.
bool refuses_a_wrong_joint_count()
{
    cartan::bench::description_spec spec = cartan::bench::description_specs().front();
    spec.joints = 7;
    try
    {
        cartan::bench::chain_from_description<7>(spec);
    }
    catch (const std::exception& refusal)
    {
        std::printf("joint-count refusal: %s\n\n", refusal.what());
        return true;
    }
    std::printf("joint-count refusal: a six-joint description was reshaped to seven\n\n");
    return false;
}

}

int main()
{
    using namespace cartan::fixtures;
    const auto& specs = cartan::bench::description_specs();
    const bool agreed =
        agrees<6>(specs[0], make_abb_irb120_chain_extended<double>(), 301ULL)
        & agrees<6>(specs[1], make_kr6_sixx_chain_extended<double>(), 300ULL)
        & agrees<7>(specs[2], make_iiwa14_chain_extended<double>(), 104ULL)
        & agrees<7>(specs[3], make_panda_chain_extended<double>(), 200ULL)
        & agrees<6>(specs[4], make_ur3e_chain_extended<double>(), 100ULL);
    return agreed & refuses_a_wrong_joint_count() ? 0 : 1;
}
