set(CARTAN_OPTIONAL_BACKENDS argmin nlopt)

set(CARTAN_ARGMIN_PACKAGE argmin)
set(CARTAN_ARGMIN_TARGET argmin::argmin)
set(CARTAN_ARGMIN_REPOSITORY https://github.com/skrede/argmin.git)
set(CARTAN_ARGMIN_REVISION 864d558c10d43337399d24f7c27cdfa0e5275475)

set(CARTAN_NLOPT_TARGET nlopt)

macro(cartan_acquire_optional_backends)
    set(CARTAN_ARGMIN_PROVIDER absent)
    set(CARTAN_NLOPT_PROVIDER absent)
    if (CARTAN_BUILD_NLOPT)
        set(CARTAN_NLOPT_PROVIDER fetched)
    endif ()
    if (CARTAN_BUILD_ARGMIN)
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
            if (argmin_FOUND)
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
    set(_cartan_pending "${CMAKE_SOURCE_DIR}")
    set(_cartan_reached "")
    while (_cartan_pending)
        list(POP_FRONT _cartan_pending _cartan_dir)
        get_property(_cartan_subdirs DIRECTORY "${_cartan_dir}" PROPERTY SUBDIRECTORIES)
        get_property(_cartan_dir_targets DIRECTORY "${_cartan_dir}" PROPERTY BUILDSYSTEM_TARGETS)
        list(APPEND _cartan_pending ${_cartan_subdirs})
        list(APPEND _cartan_reached ${_cartan_dir_targets})
    endwhile ()
    foreach (_cartan_target IN LISTS _cartan_reached)
        get_target_property(_cartan_defs ${_cartan_target} INTERFACE_COMPILE_DEFINITIONS)
        foreach (_cartan_def IN LISTS _cartan_defs)
            if (NOT _cartan_def MATCHES "^CARTAN_HAS_([A-Z0-9_]+)=")
                continue ()
            endif ()
            string(TOLOWER "${CMAKE_MATCH_1}" _cartan_name)
            if (NOT _cartan_name IN_LIST CARTAN_OPTIONAL_BACKENDS)
                message(FATAL_ERROR "target '${_cartan_target}' propagates '${_cartan_def}' for a "
                    "backend that is not registered; add '${_cartan_name}' to "
                    "CARTAN_OPTIONAL_BACKENDS instead of hand-writing a backend target")
            endif ()
            if (NOT _cartan_target STREQUAL "cartan_${_cartan_name}")
                message(FATAL_ERROR "target '${_cartan_target}' propagates '${_cartan_def}', which "
                    "only the registered component cartan_${_cartan_name} may carry")
            endif ()
        endforeach ()
    endforeach ()
endmacro()
