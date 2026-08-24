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
            if (NOT TARGET ${CARTAN_${_cartan_upper}_TARGET})
                message(FATAL_ERROR "optional backend '${_cartan_backend}': its dependency target "
                    "${CARTAN_${_cartan_upper}_TARGET} does not exist; check the registry entry "
                    "CARTAN_${_cartan_upper}_TARGET against the name the package actually creates")
            endif ()
            if (NOT "${CARTAN_${_cartan_upper}_TARGET}" IN_LIST _cartan_links)
                message(FATAL_ERROR "optional backend '${_cartan_backend}': its component does not "
                    "link ${CARTAN_${_cartan_upper}_TARGET}, so a consumer that links the component "
                    "compiles the backend headers behind CARTAN_HAS_${_cartan_upper}=1 and links "
                    "none of the backend")
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
            cartan_assert_target_definitions(${_cartan_target})
        endforeach ()
    endif ()
endmacro()

# A per-source COMPILE_DEFINITIONS never reaches the target property, so walking
# target properties alone is blind to the form that puts translation units built
# under different backend configurations into one target.
function(cartan_assert_source_definitions target)
    get_target_property(sources ${target} SOURCES)
    if (NOT sources)
        return()
    endif ()
    foreach (source IN LISTS sources)
        get_source_file_property(defs "${source}"
            TARGET_DIRECTORY ${target} COMPILE_DEFINITIONS)
        if (defs)
            cartan_assert_registered_definitions(${target} "${defs}")
        endif ()
    endforeach ()
endfunction()

function(cartan_assert_target_definitions target)
    get_target_property(own_defs ${target} COMPILE_DEFINITIONS)
    get_target_property(iface_defs ${target} INTERFACE_COMPILE_DEFINITIONS)
    set(defs "")
    list(APPEND defs ${own_defs} ${iface_defs})
    cartan_assert_registered_definitions(${target} "${defs}")
    cartan_assert_source_definitions(${target})
endfunction()

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
