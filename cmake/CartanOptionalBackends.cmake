set(CARTAN_OPTIONAL_BACKENDS argmin)

set(CARTAN_ARGMIN_PACKAGE argmin)
set(CARTAN_ARGMIN_TARGET argmin::argmin)
set(CARTAN_ARGMIN_REPOSITORY https://github.com/skrede/argmin.git)
set(CARTAN_ARGMIN_REVISION 864d558c10d43337399d24f7c27cdfa0e5275475)

macro(cartan_acquire_argmin)
    if (CARTAN_ARGMIN_SOURCE_DIR)
        FetchContent_Declare(argmin
            SOURCE_DIR "${CARTAN_ARGMIN_SOURCE_DIR}"
            EXCLUDE_FROM_ALL
            SYSTEM
        )
        # A local checkout produces an in-tree target belonging to no export
        # set, exactly as a fetch does. Reporting it as found would default the
        # install surface on, and the feasibility gate would then refuse the
        # configure outright instead of quietly generating no install rules.
        set(CARTAN_ARGMIN_PROVIDER fetched)
    else ()
        find_package(${CARTAN_ARGMIN_PACKAGE} CONFIG QUIET)
        if (${CARTAN_ARGMIN_PACKAGE}_FOUND OR TARGET ${CARTAN_ARGMIN_TARGET})
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

macro(cartan_acquire_optional_backends)
    set(CARTAN_ARGMIN_PROVIDER absent)
    if (CARTAN_BUILD_ARGMIN)
        cartan_acquire_argmin()
    endif ()
    cartan_report_optional_backend_providers()
endmacro()

function(cartan_report_optional_backend_providers)
    foreach (backend IN LISTS CARTAN_OPTIONAL_BACKENDS)
        string(TOUPPER "${backend}" upper)
        if (NOT CARTAN_BUILD_${upper})
            continue ()
        endif ()
        if (CARTAN_${upper}_PROVIDER STREQUAL "found" AND ${CARTAN_${upper}_PACKAGE}_FOUND)
            set(origin "the installed package")
        elseif (CARTAN_${upper}_PROVIDER STREQUAL "found")
            set(origin "a target the enclosing project already defined")
        elseif (CARTAN_${upper}_SOURCE_DIR)
            set(origin "the local checkout at ${CARTAN_${upper}_SOURCE_DIR}")
        else ()
            set(origin "a clone of ${CARTAN_${upper}_REPOSITORY} at ${CARTAN_${upper}_REVISION}")
        endif ()
        # A fallback that silently overrides a prefix the user supplied on the
        # command line is indistinguishable from a resolution that worked, so
        # the chosen provider is reported whether or not anything went wrong.
        message(STATUS "cartan: optional backend '${backend}' ${CARTAN_${upper}_PROVIDER} "
            "from ${origin}")
        if (NOT TARGET ${CARTAN_${upper}_TARGET})
            message(FATAL_ERROR "optional backend '${backend}' was ${CARTAN_${upper}_PROVIDER} "
                "from ${origin}, but its dependency target ${CARTAN_${upper}_TARGET} does not "
                "exist afterwards; the component would carry CARTAN_HAS_${upper}=1 while linking "
                "nothing, and an export file would name a target no consumer can resolve")
        endif ()
    endforeach ()
endfunction()

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
