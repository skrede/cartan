include_guard(GLOBAL)

# How the benchmark suite obtains what it measures against.
#
# This repository provisions only what it owns or collaborates on and what a
# reader has no other way to get. A comparator is a library anyone can install,
# so it is discovered on the machine and never downloaded, copied or patched
# here. Three acquisition classes, and the test is the same in both directions --
# can a reader obtain this themselves:
#
#   owned       the robot-description supplier and the optimization backend.
#               Nothing packages them, so they are fetched at an immutable commit
#               and the commit the build resolved is verified against the pin.
#   mainstream  every comparator. Discovered through package configuration or a
#               caller-supplied path, recorded by the version it reports, and
#               omitted with an announcement when it is not there.
#   generated   the algebraically-derived per-robot solver source. It is codegen
#               output for one robot rather than a library anyone could install,
#               so it stays in the tree; its provenance lives in its own README.
#
# The measurement harness is the one third-party dependency this build may still
# download, because it produces the measurement rather than participating in it,
# and CARTAN_FETCH_BENCHMARK_DEPS gates that and nothing else.

include(${CMAKE_CURRENT_LIST_DIR}/CartanBenchProvenance.cmake)

option(CARTAN_STRICT_COMPARATORS
    "Refuse the configure when an enabled comparator is not found, instead of omitting its cells"
    OFF)

set(CARTAN_TRAC_IK_SOURCE_DIR "" CACHE PATH
    "Path to a trac_ik_lib source checkout, for a developer who has it as sources rather than installed")
set(CARTAN_OPW_KINEMATICS_SOURCE_DIR "" CACHE PATH
    "Path to an opw_kinematics source checkout")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

# --- The measurement harness ---
find_package(benchmark CONFIG QUIET)
if (TARGET benchmark::benchmark)
    cartan_bench_refuse_unversioned(benchmark "${benchmark_VERSION}"
        "the version exported by its package configuration")
    cartan_bench_record(benchmark mainstream "the installed package" "${benchmark_VERSION}" "")
elseif (CARTAN_FETCH_BENCHMARK_DEPS)
    # v1.9.5, written as the commit that tag names.
    set(_benchmark_pin 192ef10025eb2c4cdd392bc502f0c852196baa48)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG ${_benchmark_pin}
        EXCLUDE_FROM_ALL
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
    cartan_bench_project_version("${benchmark_SOURCE_DIR}" _benchmark_version)
    cartan_bench_refuse_unversioned(benchmark "${_benchmark_version}"
        "the version its own project() call declares")
    cartan_bench_record(benchmark mainstream "a clone at ${_benchmark_pin}"
        "${_benchmark_version}" "")
else ()
    cartan_bench_absent(benchmark mainstream
        "Google Benchmark is neither installed nor enabled for fetching; install it or configure with -DCARTAN_FETCH_BENCHMARK_DEPS=ON")
endif ()

# --- The kinematics-and-dynamics library, which every benchmark target links ---
find_package(orocos_kdl CONFIG QUIET)

# Debian and its derivatives install the package configuration without the
# target file it includes, so the configuration resolves while the imported
# target it is supposed to define never appears. Its pkg-config module carries
# the same include path, library and version, so the import is rebuilt from
# there rather than from the configuration's own variables -- those name the
# absent target, and consuming them would make the replacement refer to itself.
if (NOT TARGET orocos-kdl)
    find_package(PkgConfig QUIET)
    if (PkgConfig_FOUND)
        pkg_check_modules(CARTAN_OROCOS_KDL QUIET IMPORTED_TARGET GLOBAL orocos-kdl)
        if (TARGET PkgConfig::CARTAN_OROCOS_KDL)
            add_library(orocos-kdl ALIAS PkgConfig::CARTAN_OROCOS_KDL)
            set(orocos_kdl_VERSION "${CARTAN_OROCOS_KDL_VERSION}")
        endif ()
    endif ()
endif ()

if (TARGET orocos-kdl)
    cartan_bench_refuse_unversioned(orocos_kdl "${orocos_kdl_VERSION}"
        "the version exported by its package configuration")
    cartan_bench_record(orocos_kdl mainstream "the installed package" "${orocos_kdl_VERSION}" "")
else ()
    cartan_bench_absent(orocos_kdl mainstream
        "orocos-kdl is not installed, and the shared chain factories every benchmark target uses are written against it")
endif ()

# --- The peer rigid-body-dynamics library ---
find_package(pinocchio CONFIG QUIET)
if (TARGET pinocchio::pinocchio)
    cartan_bench_refuse_unversioned(pinocchio "${pinocchio_VERSION}"
        "the version exported by its package configuration")
    cartan_bench_record(pinocchio mainstream "the installed package" "${pinocchio_VERSION}" 1)
else ()
    cartan_bench_absent(pinocchio mainstream
        "pinocchio is not installed; install it or add its prefix to CMAKE_PREFIX_PATH")
endif ()

# --- The sequential-programming library, behind the example that publishes the policy ---
find_package(NLopt CONFIG QUIET)
if (TARGET NLopt::nlopt OR NLopt_FOUND)
    cartan_bench_refuse_unversioned(nlopt "${NLopt_VERSION}"
        "the version exported by its package configuration")
    cartan_bench_record(nlopt mainstream "the installed package" "${NLopt_VERSION}" "")
else ()
    cartan_bench_absent(nlopt mainstream "NLopt is not installed")
endif ()

# --- The wall-clock-budgeted comparator. It ships no package configuration, so
# a find module serves the developer who has it installed and the source-directory
# variable serves the one who has it as a checkout. ---
if (CARTAN_TRAC_IK_SOURCE_DIR)
    cartan_bench_project_version("${CARTAN_TRAC_IK_SOURCE_DIR}" _trac_ik_version)
    cartan_bench_git_revision("${CARTAN_TRAC_IK_SOURCE_DIR}" _trac_ik_revision)
    if (_trac_ik_version STREQUAL "")
        set(_trac_ik_version "${_trac_ik_revision}")
    endif ()
    cartan_bench_refuse_unversioned(trac_ik "${_trac_ik_version}"
        "the version its own project() call declares, then the checkout's resolved revision")
    set(TRAC_IK_WITH_URDF OFF CACHE BOOL "" FORCE)
    find_package(Eigen3 CONFIG REQUIRED)
    add_subdirectory("${CARTAN_TRAC_IK_SOURCE_DIR}"
        "${CMAKE_CURRENT_BINARY_DIR}/trac_ik_lib" EXCLUDE_FROM_ALL SYSTEM)
    if (NOT TARGET TracIK::trac_ik)
        add_library(TracIK::trac_ik ALIAS trac_ik)
    endif ()
    cartan_bench_record(trac_ik mainstream
        "the source checkout at ${CARTAN_TRAC_IK_SOURCE_DIR}, revision ${_trac_ik_revision}"
        "${_trac_ik_version}" 0)
else ()
    find_package(TracIK QUIET)
    if (TARGET TracIK::trac_ik)
        cartan_bench_refuse_unversioned(trac_ik "${TracIK_VERSION}"
            "the version its installed package manifest declares")
        cartan_bench_record(trac_ik mainstream "the installed library at ${TracIK_LIBRARY}"
            "${TracIK_VERSION}" 0)
    else ()
        cartan_bench_absent(trac_ik mainstream
            "no installed trac_ik was found and CARTAN_TRAC_IK_SOURCE_DIR is unset; install it or point that variable at a checkout")
    endif ()
endif ()

# --- The ortho-parallel closed-form reference. Header-only in use: its own CMake
# expects a ROS workspace, so only its include directory is consumed. ---
if (CARTAN_OPW_KINEMATICS_SOURCE_DIR)
    cartan_bench_project_version("${CARTAN_OPW_KINEMATICS_SOURCE_DIR}" _opw_version)
    cartan_bench_git_revision("${CARTAN_OPW_KINEMATICS_SOURCE_DIR}" _opw_revision)
    if (_opw_version STREQUAL "")
        set(_opw_version "${_opw_revision}")
    endif ()
    cartan_bench_refuse_unversioned(opw_kinematics "${_opw_version}"
        "the version its own project() call declares, then the checkout's resolved revision")
    find_package(Eigen3 CONFIG REQUIRED)
    add_library(opw_kinematics_reference INTERFACE)
    target_include_directories(opw_kinematics_reference SYSTEM INTERFACE
        "${CARTAN_OPW_KINEMATICS_SOURCE_DIR}/include")
    target_link_libraries(opw_kinematics_reference INTERFACE Eigen3::Eigen)
    cartan_bench_record(opw_kinematics mainstream
        "the source checkout at ${CARTAN_OPW_KINEMATICS_SOURCE_DIR}, revision ${_opw_revision}"
        "${_opw_version}" "")
else ()
    cartan_bench_absent(opw_kinematics mainstream
        "CARTAN_OPW_KINEMATICS_SOURCE_DIR is unset; point it at a checkout to enable the ortho-parallel reference")
endif ()

# --- The linear-algebra library the generated solver calls for polynomial roots.
# It exposes its version neither through a variable the find module sets nor
# through a header macro, so the version is taken from the versioned name of the
# object that will be loaded -- a solver whose root finder cannot be identified is
# a solver whose answers cannot be attributed. ---
find_package(LAPACK QUIET)
if (LAPACK_FOUND OR TARGET LAPACK::LAPACK)
    cartan_bench_library_version("${LAPACK_LIBRARIES}" _lapack_version)
    list(GET LAPACK_LIBRARIES 0 _lapack_first)
    cartan_bench_refuse_unversioned(lapack "${_lapack_version}"
        "the version exported by its package configuration, then the versioned name of the object it resolved to")
    cartan_bench_record(lapack mainstream "the installed package at ${_lapack_first}"
        "${_lapack_version}" "")
else ()
    cartan_bench_absent(lapack mainstream
        "LAPACK is not installed, and the generated per-robot solver calls it for polynomial roots")
endif ()

# --- The geometric-decomposition solver, reached through this project's own
# foreign-function shim. The shim is cartan's code; the crate it binds is an
# ordinary registry dependency, locked in the shim's Cargo.lock. ---
set(_ikgeo_shim_dir "${CMAKE_SOURCE_DIR}/benchmarks/third_party/ikgeo_ffi")
set(_ikgeo_version "")
if (EXISTS "${_ikgeo_shim_dir}/Cargo.lock")
    file(READ "${_ikgeo_shim_dir}/Cargo.lock" _ikgeo_lock)
    string(REGEX MATCH "name = \"ik-geo\"[\r\n]+version = \"([^\"]+)\"" _matched "${_ikgeo_lock}")
    set(_ikgeo_version "${CMAKE_MATCH_1}")
endif ()
if (NOT CARTAN_IKGEO_FFI_LIB)
    find_program(CARTAN_CARGO_EXECUTABLE cargo)
endif ()
if (CARTAN_IKGEO_FFI_LIB OR CARTAN_CARGO_EXECUTABLE)
    cartan_bench_refuse_unversioned(ik_geo "${_ikgeo_version}"
        "the resolution its own Cargo.lock fixes")
    if (CARTAN_IKGEO_FFI_LIB)
        cartan_bench_record(ik_geo mainstream "a prebuilt shim at ${CARTAN_IKGEO_FFI_LIB}"
            "${_ikgeo_version}" "")
    else ()
        cartan_bench_record(ik_geo mainstream "the crate registry, built by cargo"
            "${_ikgeo_version}" "")
    endif ()
else ()
    cartan_bench_absent(ik_geo mainstream
        "neither cargo nor a prebuilt -DCARTAN_IKGEO_FFI_LIB=<path> is available, so the shim that binds the crate cannot be produced")
endif ()

# --- The owned class. Its revisions are this build's own choice, so each is
# verified against its declared pin rather than merely recorded. ---
FetchContent_GetProperties(meios SOURCE_DIR _meios_source)
if (_meios_source)
    cartan_bench_verify_pin(meios "${_meios_source}" "${CARTAN_MEIOS_REVISION}")
    cartan_bench_record(meios owned "a clone at the declared pin" "${CARTAN_MEIOS_REVISION}" "")
elseif (TARGET ${CARTAN_MEIOS_TARGET})
    cartan_bench_refuse_unversioned(meios "${meios_VERSION}"
        "the version exported by its package configuration")
    cartan_bench_record(meios owned "the installed package" "${meios_VERSION}" "")
endif ()

if (CARTAN_BUILD_ARGMIN)
    FetchContent_GetProperties(argmin SOURCE_DIR _argmin_source)
    if (_argmin_source)
        cartan_bench_verify_pin(argmin "${_argmin_source}" "${CARTAN_ARGMIN_REVISION}")
        cartan_bench_record(argmin owned "a clone at the declared pin"
            "${CARTAN_ARGMIN_REVISION}" "")
    elseif (TARGET ${CARTAN_ARGMIN_TARGET})
        cartan_bench_refuse_unversioned(argmin "${argmin_VERSION}"
            "the version exported by its package configuration")
        cartan_bench_record(argmin owned "the installed package" "${argmin_VERSION}" "")
    endif ()
endif ()

# --- The generated class, whose single member states the boundary of the
# no-copying rule: codegen output for one robot is a fixture, not a library
# anyone could install. ---
cartan_bench_record(ikfast_kr6r900 generated
    "this tree, see benchmarks/third_party/ikfast_kr6r900/README.md"
    "ikfast 0x1000004c, kinematics hash 06b2c8e082c6110f0c322529e18b8ef5" "")
