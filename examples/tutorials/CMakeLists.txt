# Student-walkthrough tutorials. Tutorial sources are thicker, prose-heavier
# descendants of the feature demos in the parent examples/ directory, and are
# registered as CTest entries so they double as build-time smoke tests.
#
# Tutorial 02 (FK + Jacobians) builds against the core cartan target only and
# is always built when CARTAN_BUILD_EXAMPLES=ON. Tutorial 01 (URDF walkthrough)
# additionally requires the URDF loading module and so is gated on
# CARTAN_BUILD_URDF=ON. The `tutorials` aggregate target depends on whichever
# tutorial executables are configured in.

add_executable(02_fk_and_jacobians 02_fk_and_jacobians.cpp)
target_link_libraries(02_fk_and_jacobians PRIVATE cartan::cartan)
add_test(NAME tutorial_02_fk_and_jacobians COMMAND 02_fk_and_jacobians)

if (CARTAN_BUILD_URDF)
    add_executable(01_urdf_walkthrough 01_urdf_walkthrough.cpp)
    target_link_libraries(01_urdf_walkthrough PRIVATE cartan::cartan cartan::urdf)
    # Inject the absolute source-tree path to the bundled synthetic URDF as a
    # compile-time string so the tutorial can be launched with zero arguments
    # from any working directory. This mirrors the CARTAN_TESTS_FIXTURE_DIR
    # pattern used throughout tests/unit/CMakeLists.txt; configure_file is
    # reserved for installed headers that participate in the install graph.
    target_compile_definitions(01_urdf_walkthrough PRIVATE
        CARTAN_TUTORIAL_DEFAULT_URDF="${CMAKE_SOURCE_DIR}/tests/fixtures/urdf/cartanbot.urdf")
    add_test(NAME tutorial_01_urdf_walkthrough COMMAND 01_urdf_walkthrough)

    add_custom_target(tutorials DEPENDS
        01_urdf_walkthrough 02_fk_and_jacobians)
else ()
    add_custom_target(tutorials DEPENDS 02_fk_and_jacobians)
endif ()
