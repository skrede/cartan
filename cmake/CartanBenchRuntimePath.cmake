include_guard(GLOBAL)

# Which C++ runtime a benchmark executable loads, when a comparator prefix ships
# one of its own.
#
# A comparator stack supplied through CMAKE_PREFIX_PATH may carry its own
# libstdc++ -- a conda or pixi environment does, a distribution's ROS prefix does
# not. CMake writes such a prefix's library directory into the build RUNPATH, and
# the dynamic loader searches RUNPATH ahead of the default directories, so a
# prefix whose runtime is older than the compiler's wins over the one the objects
# were built against and the executable dies at load on a missing symbol version.
#
# Reordering the search path is not free: a prefix that legitimately overrides a
# system library relies on being found first, and several libraries this suite
# links exist both system-wide and inside a robotics prefix. So the reorder is
# conditional on the defect being present -- an older libstdc++ actually sitting
# on a prefix -- and a prefix that ships no C++ runtime, or a newer one, is left
# in the order the caller established.

# The trailing component of a GNU libstdc++ soname, through any symlink. Ordering
# these integers orders the releases; a runtime that does not follow the scheme
# yields nothing and is treated as unknown rather than as older or newer.
function(cartan_bench_libstdcxx_revision LIBRARY OUTPUT)
    set(${OUTPUT} "" PARENT_SCOPE)
    if (NOT EXISTS "${LIBRARY}")
        return()
    endif ()
    get_filename_component(resolved "${LIBRARY}" REALPATH)
    get_filename_component(name "${resolved}" NAME)
    if (name MATCHES "libstdc\\+\\+\\.so\\.6\\.0\\.([0-9]+)")
        set(${OUTPUT} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    endif ()
endfunction()

function(cartan_bench_toolchain_runtime OUTPUT)
    set(${OUTPUT} "" PARENT_SCOPE)
    if (NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        return()
    endif ()
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++.so.6
        OUTPUT_VARIABLE library
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if (IS_ABSOLUTE "${library}" AND EXISTS "${library}")
        set(${OUTPUT} "${library}" PARENT_SCOPE)
    endif ()
endfunction()

# The first prefix carrying a C++ runtime older than the toolchain's. Search is
# bounded to the prefixes the caller named, which is where a foreign runtime
# arrives from; a library pulled in by some other mechanism is not covered, and
# the load-time error names the file when that happens.
function(cartan_bench_outdated_prefix REVISION OUTPUT)
    set(${OUTPUT} "" PARENT_SCOPE)
    foreach (prefix IN LISTS CMAKE_PREFIX_PATH CMAKE_LIBRARY_PATH)
        foreach (suffix lib lib64)
            cartan_bench_libstdcxx_revision(
                "${prefix}/${suffix}/libstdc++.so.6" prefix_revision)
            if (prefix_revision AND prefix_revision LESS REVISION)
                set(${OUTPUT} "${prefix}/${suffix}" PARENT_SCOPE)
                return()
            endif ()
        endforeach ()
    endforeach ()
endfunction()

# Read at target creation, so this has to precede every add_executable in the
# directory that includes it.
function(cartan_bench_prefer_toolchain_runtime)
    cartan_bench_toolchain_runtime(runtime)
    cartan_bench_libstdcxx_revision("${runtime}" revision)
    if (NOT revision)
        return()
    endif ()
    cartan_bench_outdated_prefix("${revision}" outdated)
    if (NOT outdated)
        return()
    endif ()
    get_filename_component(runtime_dir "${runtime}" DIRECTORY)
    get_filename_component(runtime_dir "${runtime_dir}" REALPATH)
    set(rpath "${runtime_dir}" ${CMAKE_BUILD_RPATH})
    list(REMOVE_DUPLICATES rpath)
    set(CMAKE_BUILD_RPATH "${rpath}" PARENT_SCOPE)
    message(STATUS "cartan benchmarks: ${outdated} carries an older C++ runtime than the compiler, "
        "so ${runtime_dir} is searched ahead of it")
endfunction()
