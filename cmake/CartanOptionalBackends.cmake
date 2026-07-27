set(CARTAN_OPTIONAL_BACKENDS argmin nlopt)

set(CARTAN_ARGMIN_PACKAGE argmin)
set(CARTAN_ARGMIN_TARGET argmin::argmin)
set(CARTAN_ARGMIN_REPOSITORY https://github.com/skrede/argmin.git)
set(CARTAN_ARGMIN_REVISION 864d558c10d43337399d24f7c27cdfa0e5275475)

set(CARTAN_NLOPT_PACKAGE NLopt)
set(CARTAN_NLOPT_TARGET NLopt::nlopt)
set(CARTAN_NLOPT_REPOSITORY https://github.com/stevengj/nlopt.git)
set(CARTAN_NLOPT_REVISION v2.10.1)

macro(cartan_acquire_argmin)
    if (CARTAN_ARGMIN_SOURCE_DIR)
        FetchContent_Declare(argmin
            SOURCE_DIR "${CARTAN_ARGMIN_SOURCE_DIR}"
            EXCLUDE_FROM_ALL
            SYSTEM
        )
        # A local checkout produces an in-tree target belonging to no export
        # set, exactly as a fetch does; reporting it as found would carry an
        # install-mode configure past the feasibility gate and into a raw
        # generate-time failure.
        set(CARTAN_ARGMIN_PROVIDER fetched)
    else ()
        find_package(${CARTAN_ARGMIN_PACKAGE} CONFIG QUIET)
        if (TARGET ${CARTAN_ARGMIN_TARGET})
            set(CARTAN_ARGMIN_PROVIDER found)
        else ()
            FetchContent_Declare(argmin
                GIT_REPOSITORY ${CARTAN_ARGMIN_REPOSITORY}
                GIT_TAG ${CARTAN_ARGMIN_REVISION}
                EXCLUDE_FROM_ALL
                SYSTEM
            )
            set(CARTAN_ARGMIN_PROVIDER fetched)
        endif ()
    endif ()
    if (CARTAN_ARGMIN_PROVIDER STREQUAL "fetched")
        block()
            set(ARGMIN_BUILD_TESTS OFF)
            set(ARGMIN_BUILD_EXAMPLES OFF)
            set(ARGMIN_BUILD_BENCHMARKS OFF)
            if (CARTAN_CMAKE_FETCH_DEPS AND NOT TARGET Eigen3::Eigen)
                set(ARGMIN_CMAKE_FETCH_DEPS ON)
            endif ()
            FetchContent_MakeAvailable(argmin)
        endblock()
    endif ()
endmacro()

macro(cartan_acquire_nlopt)
    find_package(${CARTAN_NLOPT_PACKAGE} CONFIG QUIET)
    if (TARGET ${CARTAN_NLOPT_TARGET})
        set(CARTAN_NLOPT_PROVIDER found)
    else ()
        FetchContent_Declare(nlopt
            GIT_REPOSITORY ${CARTAN_NLOPT_REPOSITORY}
            GIT_TAG ${CARTAN_NLOPT_REVISION}
            EXCLUDE_FROM_ALL
            SYSTEM
        )
        block()
            set(NLOPT_PYTHON OFF)
            set(NLOPT_OCTAVE OFF)
            set(NLOPT_GUILE OFF)
            set(NLOPT_TESTS OFF)
            set(BUILD_SHARED_LIBS OFF)
            FetchContent_MakeAvailable(nlopt)
        endblock()
        if (NOT TARGET ${CARTAN_NLOPT_TARGET})
            add_library(${CARTAN_NLOPT_TARGET} ALIAS nlopt)
        endif ()
        set(CARTAN_NLOPT_PROVIDER fetched)
    endif ()
endmacro()

macro(cartan_acquire_optional_backends)
    set(CARTAN_ARGMIN_PROVIDER absent)
    set(CARTAN_NLOPT_PROVIDER absent)
    if (CARTAN_BUILD_ARGMIN)
        cartan_acquire_argmin()
    endif ()
    if (CARTAN_BUILD_NLOPT)
        cartan_acquire_nlopt()
    endif ()
endmacro()

macro(cartan_define_optional_backend_components)
    foreach (_cartan_backend IN LISTS CARTAN_OPTIONAL_BACKENDS)
        string(TOUPPER "${_cartan_backend}" _cartan_upper)
        if (CARTAN_BUILD_${_cartan_upper})
            add_library(cartan_${_cartan_backend} INTERFACE)
            add_library(cartan::${_cartan_backend} ALIAS cartan_${_cartan_backend})

            target_link_libraries(cartan_${_cartan_backend}
                INTERFACE
                cartan::serial_chain
                ${CARTAN_${_cartan_upper}_TARGET}
            )

            target_compile_definitions(cartan_${_cartan_backend}
                INTERFACE
                CARTAN_HAS_${_cartan_upper}=1
            )

            set_target_properties(cartan_${_cartan_backend} PROPERTIES
                EXPORT_NAME ${_cartan_backend})

            if (CARTAN_ENABLE_INSTALL)
                install(TARGETS cartan_${_cartan_backend} EXPORT cartanTargets)
            endif ()
        endif ()
    endforeach ()
endmacro()

macro(cartan_assert_optional_backend_contract)
    foreach (_cartan_backend IN LISTS CARTAN_OPTIONAL_BACKENDS)
        string(TOUPPER "${_cartan_backend}" _cartan_upper)
        if (CARTAN_BUILD_${_cartan_upper})
            if (NOT TARGET cartan_${_cartan_backend} OR NOT TARGET cartan::${_cartan_backend})
                message(FATAL_ERROR "optional backend '${_cartan_backend}' is enabled but its "
                    "component target cartan_${_cartan_backend} or its alias does not exist")
            endif ()
            get_target_property(_cartan_export cartan_${_cartan_backend} EXPORT_NAME)
            get_target_property(_cartan_defs cartan_${_cartan_backend} INTERFACE_COMPILE_DEFINITIONS)
            get_target_property(_cartan_links cartan_${_cartan_backend} INTERFACE_LINK_LIBRARIES)
            if (NOT _cartan_export STREQUAL "${_cartan_backend}")
                message(FATAL_ERROR "optional backend '${_cartan_backend}': its component needs "
                    "EXPORT_NAME ${_cartan_backend} to survive install(EXPORT), found '${_cartan_export}'")
            endif ()
            if (NOT "CARTAN_HAS_${_cartan_upper}=1" IN_LIST _cartan_defs)
                message(FATAL_ERROR "optional backend '${_cartan_backend}': its component does not "
                    "propagate CARTAN_HAS_${_cartan_upper}=1 through its interface")
            endif ()
            if (NOT "cartan::serial_chain" IN_LIST _cartan_links)
                message(FATAL_ERROR "optional backend '${_cartan_backend}': its component does not "
                    "link cartan::serial_chain")
            endif ()
        endif ()
    endforeach ()
endmacro()

macro(cartan_assert_no_unregistered_backend)
    # Rooted at cartan's own tree, and skipped when cartan is embedded: an
    # embedding host's targets are not cartan's to police, and the walk would
    # otherwise abort the host's configure with cartan's remedy.
    if (PROJECT_IS_TOP_LEVEL)
        set(_cartan_pending "${PROJECT_SOURCE_DIR}")
        set(_cartan_reached "")
        while (_cartan_pending)
            list(POP_FRONT _cartan_pending _cartan_dir)
            get_property(_cartan_subdirs DIRECTORY "${_cartan_dir}" PROPERTY SUBDIRECTORIES)
            get_property(_cartan_dir_targets DIRECTORY "${_cartan_dir}" PROPERTY BUILDSYSTEM_TARGETS)
            list(APPEND _cartan_pending ${_cartan_subdirs})
            list(APPEND _cartan_reached ${_cartan_dir_targets})
        endwhile ()
        foreach (_cartan_target IN LISTS _cartan_reached)
            get_target_property(_cartan_own_defs ${_cartan_target} COMPILE_DEFINITIONS)
            get_target_property(_cartan_iface_defs ${_cartan_target} INTERFACE_COMPILE_DEFINITIONS)
            set(_cartan_defs "")
            list(APPEND _cartan_defs ${_cartan_own_defs} ${_cartan_iface_defs})
            cartan_assert_registered_definitions(${_cartan_target} "${_cartan_defs}")
        endforeach ()
    endif ()
endmacro()

function(cartan_assert_registered_definitions target definitions)
    foreach (definition IN LISTS definitions)
        if (NOT definition MATCHES "^CARTAN_HAS_([A-Z0-9_]+)(=|$)")
            continue ()
        endif ()
        string(TOLOWER "${CMAKE_MATCH_1}" name)
        if (NOT name IN_LIST CARTAN_OPTIONAL_BACKENDS)
            message(FATAL_ERROR "target '${target}' defines '${definition}' for a backend that is "
                "not registered; add '${name}' to CARTAN_OPTIONAL_BACKENDS instead of hand-writing "
                "a backend target, or spell the macro outside the CARTAN_HAS_ namespace if it does "
                "not name an optional backend")
        endif ()
        if (NOT target STREQUAL "cartan_${name}")
            message(FATAL_ERROR "target '${target}' defines '${definition}', which only the "
                "registered component cartan_${name} may carry; link cartan::${name} instead")
        endif ()
    endforeach ()
endfunction()
