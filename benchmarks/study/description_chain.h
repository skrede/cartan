#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_DESCRIPTION_CHAIN_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_DESCRIPTION_CHAIN_H

/// @file description_chain.h
/// @brief The study's feasible set, read off upstream robot descriptions.
///
/// Every joint bound the study uses comes from a description fetched at a
/// commit the build files name, loaded through cartan::load_urdf, and converted
/// here into the fixed-joint-count chain the harness measures.

#include <cartan/urdf.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <meios/eval/python_evaluator.h>

#include <meios/xacro/evaluator_handle.h>

#include <map>
#include <array>
#include <string>
#include <memory>
#include <utility>
#include <stdexcept>
#include <filesystem>
#include <string_view>
#include <type_traits>

namespace cartan::bench
{

/// One robot's description. repository_key is the checked-out repository's
/// directory under CARTAN_BENCH_DESCRIPTION_ROOT, and relative_path is the
/// document's path under that same root.
struct description_spec
{
    std::string_view repository_key;
    std::string_view relative_path;
    std::map<std::string, std::string> args;
    int joints;
    std::string_view robot_key;
};

/// Both UR arguments are load-bearing: without them the description falls back
/// to its own ur5x default, a placeholder variant shipping no joint-limits
/// file, and the load fails on a path that is not there.
inline const std::array<description_spec, 9>& description_specs()
{
    static const std::array<description_spec, 9> specs{
        description_spec{
            "abb", "abb/abb_irb120_support/urdf/irb120_3_58.xacro", {}, 6, "abb_irb120"},
        description_spec{
            "kuka_experimental", "kuka_experimental/kuka_kr6_support/urdf/kr6r900sixx.xacro",
            {}, 6, "kuka_kr6_r900"},
        description_spec{
            "kuka_experimental",
            "kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf",
            {}, 7, "kuka_lbr_iiwa14_r820"},
        description_spec{
            "lbr_med14_r820_description",
            "lbr_med14_r820_description/urdf/lbr_med14_r820.urdf.xacro",
            {}, 7, "kuka_lbr_med14_r820"},
        description_spec{
            "franka_ros", "franka_ros/franka_description/robots/panda/panda.urdf.xacro",
            {}, 7, "franka_panda"},
        description_spec{
            "ur_description", "ur_description/urdf/ur.urdf.xacro",
            {{"ur_type", "ur3e"}, {"name", "ur3e"}}, 6, "universal_robots_ur3e"},
        description_spec{
            "ur_description", "ur_description/urdf/ur.urdf.xacro",
            {{"ur_type", "ur5e"}, {"name", "ur5e"}}, 6, "universal_robots_ur5e"},
        description_spec{
            "ur_description", "ur_description/urdf/ur.urdf.xacro",
            {{"ur_type", "ur10"}, {"name", "ur10"}}, 6, "universal_robots_ur10"},
        description_spec{
            "ur_description", "ur_description/urdf/ur.urdf.xacro",
            {{"ur_type", "ur16e"}, {"name", "ur16e"}}, 6, "universal_robots_ur16e"}};
    return specs;
}

inline const description_spec& description_for(std::string_view robot)
{
    const description_spec* found = nullptr;
    for (const auto& spec : description_specs())
    {
        if (spec.robot_key == robot || spec.robot_key.ends_with(robot))
        {
            if (found != nullptr)
            {
                throw std::runtime_error(std::string{robot} + ": names more than one description");
            }
            found = &spec;
        }
    }
    if (found == nullptr)
    {
        throw std::runtime_error(std::string{robot} + ": the study carries no such description");
    }
    return *found;
}

/// The restricted evaluator is the widest authority anything under benchmarks/
/// grants a description. The UR and Franka documents read their joint limits
/// with a host-language call, so an evaluator is unavoidable; the backend that
/// applies none of the refusal rules would evaluate every expression an include
/// pulls in with the process's own authority, and it has been measured to
/// produce the same result on these inputs, so widening buys nothing.
///
/// Two roots because the repositories carry their packages at two depths: the
/// description root reaches a repository that is itself one package, and the
/// repository directory reaches the packages a multi-package repository holds.
inline cartan::load_options description_load_options(const description_spec& spec)
{
    const std::filesystem::path root{CARTAN_BENCH_DESCRIPTION_ROOT};
    cartan::load_options opts;
    opts.description.backend =
        std::make_shared<meios::evaluator_handle>(meios::python_evaluator{});
    opts.description.package_roots.push_back(root);
    opts.description.package_roots.push_back(root / spec.repository_key);
    opts.description.args = spec.args;
    return opts;
}

namespace detail
{

template <int N, std::size_t... I>
cartan::kinematic_chain<double, N> fixed_from_dynamic(
    const cartan::kinematic_chain<double, cartan::dynamic>& chain,
    std::index_sequence<I...>)
{
    return cartan::kinematic_chain<double, N>(
        chain.home(), {chain.axes()[I]...}, {chain.limits()[I]...});
}

}

/// The harness measures a compile-time joint count and the loader returns a
/// runtime one, so a spec whose declared count disagrees with the description's
/// is refused here: reshaping it would hand the study a robot that is not the
/// one on disk.
template <int N>
cartan::kinematic_chain<double, N> chain_from_description(const description_spec& spec)
{
    const std::filesystem::path root{CARTAN_BENCH_DESCRIPTION_ROOT};
    auto loaded = cartan::load_urdf<double>(root / spec.relative_path, description_load_options(spec));
    if (!loaded)
    {
        throw std::runtime_error(
            std::string{spec.robot_key} + ": description refused: " + loaded.error().detail);
    }
    if (loaded->chain.num_joints() != N)
    {
        throw std::runtime_error(
            std::string{spec.robot_key} + ": the description declares "
            + std::to_string(loaded->chain.num_joints()) + " joints, the study expects "
            + std::to_string(N));
    }
    return detail::fixed_from_dynamic<N>(
        loaded->chain, std::make_index_sequence<static_cast<std::size_t>(N)>{});
}

/// Which joint count a program instantiates is the description's own declared
/// one rather than a flag, so a run cannot be pointed at a robot and measured as
/// though it were a different shape.
template <typename Run>
int dispatch_on_joints(const description_spec& spec, Run&& run)
{
    if (spec.joints == 6)
    {
        return run(std::integral_constant<int, 6>{});
    }
    if (spec.joints == 7)
    {
        return run(std::integral_constant<int, 7>{});
    }
    throw std::runtime_error(std::string{spec.robot_key}
        + ": the study measures six- and seven-axis arms, and this description declares "
        + std::to_string(spec.joints) + " joints");
}

}

#endif
