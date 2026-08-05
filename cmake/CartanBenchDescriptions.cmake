include_guard(GLOBAL)

# Every revision below is a commit, never a branch and never a release tag. A
# moving reference lets a later configure fetch a different robot under an
# unchanged benchmark name, and the joint bounds the study calls the
# manufacturer's would then have no fixed referent. The clones are deliberately
# not shallow: a shallow fetch retrieves only branch and tag tips, and none of
# these pins is either. SOURCE_SUBDIR names a path that does not exist so the
# populated sources are never configured -- these are ROS packages whose own
# CMake expects an ament workspace, and the study reads their description
# documents off disk.
#
# Each repository is checked out directly under the description root, under the
# directory name the study's specs address it by. The reader resolves
# package://<name>/<path> as <root>/<name>/<path> against a canonicalized root,
# so neither a symlink farm nor a deeper search can stand in for a checkout that
# is already in the right place.

set(CARTAN_BENCH_DESCRIPTION_ROOT "${CMAKE_BINARY_DIR}/bench-descriptions" CACHE PATH
    "Directory the fetched robot description repositories are checked out under" FORCE)

FetchContent_Declare(
    abb_descriptions
    GIT_REPOSITORY https://github.com/ros-industrial/abb.git
    GIT_TAG 45f4769d826cf3ac62a65495f2db67b78b0c81df
    SOURCE_DIR "${CARTAN_BENCH_DESCRIPTION_ROOT}/abb"
    SOURCE_SUBDIR does-not-exist
    EXCLUDE_FROM_ALL
    SYSTEM
)

FetchContent_Declare(
    kuka_descriptions
    GIT_REPOSITORY https://github.com/ros-industrial/kuka_experimental.git
    GIT_TAG 8d9292b04a22628b1b78d989e2ddd3abb913bf92
    SOURCE_DIR "${CARTAN_BENCH_DESCRIPTION_ROOT}/kuka_experimental"
    SOURCE_SUBDIR does-not-exist
    EXCLUDE_FROM_ALL
    SYSTEM
)

FetchContent_Declare(
    franka_descriptions
    GIT_REPOSITORY https://github.com/frankaemika/franka_ros.git
    GIT_TAG c11f000c2737acc22ed8dfeff40555b2644a4294
    SOURCE_DIR "${CARTAN_BENCH_DESCRIPTION_ROOT}/franka_ros"
    SOURCE_SUBDIR does-not-exist
    EXCLUDE_FROM_ALL
    SYSTEM
)

# This repository is itself the ur_description package -- its manifest sits at
# the repository root -- so the checkout is named for the package, which is what
# package://ur_description/... resolves against under the root.
#
# Release 4.3.1 is a lightweight tag: `git ls-remote --tags <url> '4.3.1^{}'`
# peels to nothing, so the tag ref itself names the commit written here.
FetchContent_Declare(
    universal_robots_descriptions
    GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
    GIT_TAG ae333289875f9ba5a9ea6649a54036efb5ccabee
    SOURCE_DIR "${CARTAN_BENCH_DESCRIPTION_ROOT}/ur_description"
    SOURCE_SUBDIR does-not-exist
    EXCLUDE_FROM_ALL
    SYSTEM
)

# Writing a commit above once is not what keeps it a commit; this re-reads the
# declarations on every configure so an edit that swaps one for a branch is
# refused where it was made.
function(cartan_bench_assert_commit_pins)
    file(STRINGS "${CMAKE_CURRENT_FUNCTION_LIST_FILE}" declarations REGEX "^ +GIT_(REPOSITORY|TAG) ")
    set(repository "")
    foreach (declaration IN LISTS declarations)
        string(REGEX REPLACE "^ +GIT_[A-Z]+ +" "" value "${declaration}")
        if (declaration MATCHES "GIT_REPOSITORY")
            set(repository "${value}")
            continue()
        endif ()
        string(LENGTH "${value}" width)
        if (NOT width EQUAL 40 OR NOT value MATCHES "^[0-9a-f]+$")
            message(FATAL_ERROR
                "cartan benchmarks: the robot description at ${repository} is pinned at "
                "'${value}', which is not a 40-character commit. A branch or a release tag "
                "moves, and the study's joint bounds would then have no fixed referent.")
        endif ()
    endforeach ()
endfunction()

cartan_bench_assert_commit_pins()

FetchContent_MakeAvailable(
    abb_descriptions kuka_descriptions franka_descriptions universal_robots_descriptions)

message(STATUS
    "cartan benchmarks: robot descriptions resolve under ${CARTAN_BENCH_DESCRIPTION_ROOT}")
