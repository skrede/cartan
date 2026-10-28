/// @file study_description_gate.cpp
/// @brief Proves the description-loaded chains are the robots they replace.
///
/// The study's feasible set moved from a synthetic symmetric box to the bounds
/// an upstream description declares. A description for the wrong variant of a
/// robot is otherwise indistinguishable from the right one, so each loaded
/// chain is checked against the hand-coded chain the previous study measured by
/// forward-kinematics agreement, and the measured maximum is printed.

#include "study/fk_agreement.h"
#include "study/description_chain.h"

#include <string>
#include <cstdio>
#include <exception>
#include <string_view>

namespace
{

constexpr double agreement = 1e-9;

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
bool agrees(const cartan::bench::description_spec& spec)
{
    try
    {
        const auto& truth = cartan::bench::truth_for(spec.robot_key);
        const auto loaded = cartan::bench::chain_from_description<N>(spec);
        print_bounds(spec.robot_key, loaded);
        const auto [position, orientation] =
            cartan::bench::worst_fk_deviation<N>(loaded, truth.chain, truth.seed);
        std::printf("%s: max FK deviation over %d configurations %.3e m, %.3e rad\n\n",
            std::string(spec.robot_key).c_str(), cartan::bench::fk_agreement_configurations,
            position, orientation);
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
    bool agreed = true;
    for (const auto& spec : cartan::bench::description_specs())
    {
        const int robot_agreed = cartan::bench::dispatch_on_joints(spec,
            [&spec](auto joints) { return static_cast<int>(agrees<decltype(joints)::value>(spec)); });
        agreed = (robot_agreed != 0) && agreed;
    }
    return agreed & refuses_a_wrong_joint_count() ? 0 : 1;
}
