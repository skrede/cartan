if (CARTAN_BUILD_TESTS)
    if (CARTAN_CMAKE_FETCH_DEPS)
        FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.7.1
            EXCLUDE_FROM_ALL
            SYSTEM
        )
        FetchContent_MakeAvailable(Catch2)
    else ()
        find_package(Catch2 CONFIG REQUIRED)
    endif ()

    # AppleClang matches "Clang" but lacks -Wno-c2y-extensions, so probe support
    # rather than the compiler name. -Werror=unknown-warning-option makes the
    # probe fail on compilers that reject the flag.
    include(CheckCXXCompilerFlag)
    set(CMAKE_REQUIRED_FLAGS "-Werror=unknown-warning-option")
    check_cxx_compiler_flag("-Wno-c2y-extensions" CARTAN_HAS_WNO_C2Y_EXTENSIONS)
    unset(CMAKE_REQUIRED_FLAGS)

    if (CARTAN_BUILD_PROPERTY_TESTS)
        FetchContent_Declare(
            rapidcheck
            GIT_REPOSITORY https://github.com/emil-e/rapidcheck.git
            GIT_TAG b96a4e626ef4c7348dcd16c500353c2f997a9f3f
            EXCLUDE_FROM_ALL
            SYSTEM
        )
        FetchContent_MakeAvailable(rapidcheck)
        add_subdirectory(${rapidcheck_SOURCE_DIR}/extras/catch ${rapidcheck_BINARY_DIR}/extras/catch)
    endif ()

    if (CARTAN_BUILD_PROPERTY_TESTS)
        add_subdirectory(property)
    endif ()

    add_subdirectory(unit)
    add_subdirectory(compile)

    if (NOT CARTAN_CMAKE_FETCH_DEPS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/integration/CMakeLists.txt")
        add_subdirectory(integration)
    endif ()
endif ()

if (CARTAN_BUILD_FUZZ_TESTS)
    if (NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        message(FATAL_ERROR "Fuzz targets require Clang with libFuzzer support")
    endif ()
    add_subdirectory(fuzz)
endif ()
