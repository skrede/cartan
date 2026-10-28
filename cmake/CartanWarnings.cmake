set(CARTAN_WARNING_FLAGS
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /utf-8>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:
    -fPIC
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wold-style-cast
    -Wcast-align
    -Woverloaded-virtual
    -Wnon-virtual-dtor
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wformat=2
    >
)

# Off by default for a project that consumes cartan: this warning set tracks
# whatever the compilers cartan is developed against report, so a diagnostic new
# to one of them would otherwise turn into a build failure in a tree nobody here
# can see or fix.
option(CARTAN_WERROR "Treat warnings as errors on cartan's own targets" ${cartan_IS_TOP_LEVEL})

# The promotion belongs on the flag list rather than in CMAKE_CXX_FLAGS because
# that variable governs every target in the build tree, including dependencies
# fetched into it, and a diagnostic in code cartan does not own can then halt
# cartan's build with nothing cartan can do about it.
if (CARTAN_WERROR)
    list(APPEND CARTAN_WARNING_FLAGS
        $<$<CXX_COMPILER_ID:MSVC>:/WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror>
    )
endif ()

function(cartan_target_warnings target)
    target_compile_options(${target} PRIVATE ${CARTAN_WARNING_FLAGS})
endfunction()

# BUILDSYSTEM_TARGETS names the targets defined in one directory and does not
# descend into subdirectories, which is what keeps this sweep off the code a
# FetchContent_MakeAvailable call populates below a directory cartan owns. A
# recursive walk in its place would put cartan's flags on its dependencies.
function(cartan_warn_directory_targets)
    get_property(cartan_directory_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    foreach (cartan_target IN LISTS cartan_directory_targets)
        get_target_property(cartan_target_type ${cartan_target} TYPE)
        if (cartan_target_type STREQUAL "INTERFACE_LIBRARY" OR cartan_target_type STREQUAL "UTILITY")
            continue ()
        endif ()
        get_target_property(cartan_target_foreign ${cartan_target} CARTAN_THIRD_PARTY_SOURCES)
        if (cartan_target_foreign)
            continue ()
        endif ()
        cartan_target_warnings(${cartan_target})
    endforeach ()
endfunction()

function(cartan_exclude_from_warnings target)
    set_target_properties(${target} PROPERTIES CARTAN_THIRD_PARTY_SOURCES ON)
endfunction()
