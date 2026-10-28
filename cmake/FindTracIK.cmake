# Find module for trac_ik_lib, which ships no package configuration of its own.
#
# Defines the imported target TracIK::trac_ik, and sets TracIK_FOUND,
# TracIK_INCLUDE_DIR, TracIK_LIBRARY and TracIK_VERSION. The version is read from
# the ROS package manifest the library installs beside its headers, because the
# library exposes no version macro; a prefix that carries no manifest yields no
# version, and the caller decides what that means.

find_path(TracIK_INCLUDE_DIR
    NAMES trac_ik/trac_ik.hpp
    PATH_SUFFIXES include)

find_library(TracIK_LIBRARY NAMES trac_ik trac_ik_lib)

set(TracIK_VERSION "")
if (TracIK_INCLUDE_DIR)
    get_filename_component(_trac_ik_prefix "${TracIK_INCLUDE_DIR}" DIRECTORY)
    foreach (_trac_ik_manifest
        "${_trac_ik_prefix}/share/trac_ik_lib/package.xml"
        "${_trac_ik_prefix}/share/trac_ik/package.xml")
        if (EXISTS "${_trac_ik_manifest}")
            file(READ "${_trac_ik_manifest}" _trac_ik_declaration)
            string(REGEX MATCH "<version>([^<]+)</version>" _matched "${_trac_ik_declaration}")
            set(TracIK_VERSION "${CMAKE_MATCH_1}")
            break ()
        endif ()
    endforeach ()
endif ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TracIK
    REQUIRED_VARS TracIK_INCLUDE_DIR TracIK_LIBRARY)

if (TracIK_FOUND AND NOT TARGET TracIK::trac_ik)
    find_package(orocos_kdl CONFIG QUIET)
    find_package(NLopt CONFIG QUIET)
    add_library(TracIK::trac_ik INTERFACE IMPORTED)
    set_target_properties(TracIK::trac_ik PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${TracIK_INCLUDE_DIR}"
        INTERFACE_INCLUDE_DIRECTORIES "${TracIK_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${TracIK_LIBRARY}")
    foreach (_trac_ik_dependency orocos-kdl NLopt::nlopt)
        if (TARGET ${_trac_ik_dependency})
            set_property(TARGET TracIK::trac_ik APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES ${_trac_ik_dependency})
        endif ()
    endforeach ()
endif ()

mark_as_advanced(TracIK_INCLUDE_DIR TracIK_LIBRARY)
