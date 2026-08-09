/// @file unwrapped_result_test.cpp
/// @brief A multi-solution analytical result is traversed over its populated
///        subset alone, and the per-solution range verdicts stay index-parallel
///        to the solutions across that subset.
///
/// Constructing the type proves nothing about either property: the storage is
/// always the full per-solver bound, and it is end() -- offset by a runtime
/// count rather than by that bound -- which decides where a caller stops.

#include <cartan/analytical/unwrapped_result.h>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>

namespace spp = cartan;

namespace
{

constexpr int k_max_solutions = 4;

using result_t = spp::unwrapped_result<double, 2, k_max_solutions>;
using position_t = result_t::position_type;

// Slot i carries the position (i + 1, 0) and a verdict that alternates with i,
// so a traversal reading the wrong slot cannot agree with the index it came
// from by accident.
position_t solution_at(int index)
{
    return position_t(static_cast<double>(index) + 1.0, 0.0);
}

spp::range_status tag_at(int index)
{
    return index % 2 == 0 ? spp::range_status::in_range
                          : spp::range_status::joint_limits_violated;
}

result_t populated(int count)
{
    result_t result;
    for (int i = 0; i < count; ++i)
    {
        result.solutions[static_cast<std::size_t>(i)] = solution_at(i);
        result.tags[static_cast<std::size_t>(i)] = tag_at(i);
    }
    result.count = count;
    return result;
}

}

TEST_CASE("a partly populated result visits its populated prefix and stops there",
    "[analytical][unwrapped_result]")
{
    const result_t result = populated(2);

    int visited = 0;
    for (const position_t& q : result)
    {
        INFO("visited index " << visited);
        CHECK((q - solution_at(visited)).norm() < 1e-12);
        ++visited;
    }

    CHECK(visited == 2);
}

TEST_CASE("a result that carries no solution visits nothing",
    "[analytical][unwrapped_result]")
{
    const result_t result = populated(0);

    CHECK(result.begin() == result.end());
}

// The adjacency case: the populated subset and the storage are the same size,
// so the traversal must stop at the array bound and not one past it.
TEST_CASE("a fully populated result visits every slot and stops at the storage bound",
    "[analytical][unwrapped_result]")
{
    const result_t result = populated(k_max_solutions);

    int visited = 0;
    for (const position_t& q : result)
    {
        INFO("visited index " << visited);
        CHECK((q - solution_at(visited)).norm() < 1e-12);
        ++visited;
    }

    CHECK(visited == k_max_solutions);
    CHECK(result.end() == result.solutions.end());
}

// The traversal exposes the solutions alone, so the position a caller counts
// through the range is the position it must read the verdict at; a verdict read
// one slot over would report a branch inside the joint limits as outside them.
TEST_CASE("the verdicts stay index-parallel to the solutions across the populated subset",
    "[analytical][unwrapped_result]")
{
    const result_t result = populated(3);

    std::size_t index = 0;
    for (const position_t& q : result)
    {
        INFO("populated index " << index);
        CHECK((q - solution_at(static_cast<int>(index))).norm() < 1e-12);
        CHECK(result.tags[index] == tag_at(static_cast<int>(index)));
        ++index;
    }

    CHECK(index == 3);
}
