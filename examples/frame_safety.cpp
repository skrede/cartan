/// @file frame_safety.cpp
/// @brief Demonstrates compile-time frame safety with transform and rotation.
///
/// Shows: frame tag structs, transform composition, rotation composition,
/// and how frame mismatches are caught at compile time.

#include "cartan/lie.h"

#include <iostream>

struct World {};
struct Base {};
struct Tool {};

int main()
{
    using T_wb = cartan::transform<World, Base>;
    using T_bt = cartan::transform<Base, Tool>;

    // Create transforms from SE(3) values
    auto world_base = T_wb{cartan::se3<double>::exp({0, 0, 0, 1.0, 0, 0.5})};
    auto base_tool = T_bt{cartan::se3<double>::exp({0, 0, 0.3, 0.1, 0.2, 0})};

    // Valid: World->Base * Base->Tool = World->Tool
    auto world_tool = world_base * base_tool;
    std::cout << "World->Tool translation: "
              << world_tool.translation().transpose() << "\n";

    // Would NOT compile (frame mismatch):
    // auto bad = world_base * world_tool;  // Base != World

    // Inverse flips frame tags: World->Base -> Base->World
    auto base_world = world_base.inverse();

    // Rotation frame safety works the same way
    using R_wb = cartan::rotation<World, Base>;
    using R_bt = cartan::rotation<Base, Tool>;

    auto rot_wb = R_wb{cartan::so3<double>::exp({0.1, 0.2, 0.3})};
    auto rot_bt = R_bt{cartan::so3<double>::exp({0.0, 0.0, 0.5})};

    // Valid: World->Base * Base->Tool = World->Tool
    auto rot_wt = rot_wb * rot_bt;
    std::cout << "World->Tool rotation log: "
              << rot_wt.log().transpose() << "\n";
}
