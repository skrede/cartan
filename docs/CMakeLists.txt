# Documentation snippet compile gate.
#
# The guides carry HTML-comment sentinels marking which fenced ``cpp`` blocks are
# real, buildable code. tools/extract_doc_snippets.py turns each marked block into
# a standalone translation unit; those units are compiled here against the actual
# public headers. A renamed symbol or a wrong signature in the prose therefore
# breaks the build instead of rotting silently.
#
# These targets are compile-only: they are never linked into an executable and
# never run. Their sole purpose is to force the compiler across the documented API.

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_doc_snippet_extractor ${CMAKE_SOURCE_DIR}/tools/extract_doc_snippets.py)
set(_doc_snippet_gen_dir ${CMAKE_CURRENT_BINARY_DIR}/doc_snippets)

# Regenerate the snippets at configure time. A malformed sentinel makes the
# extractor exit non-zero, which we promote to a hard configure error so the gate
# fails loudly rather than silently dropping a snippet.
execute_process(
    COMMAND ${Python3_EXECUTABLE} ${_doc_snippet_extractor}
            --docs-root ${CMAKE_CURRENT_SOURCE_DIR}
            --out ${_doc_snippet_gen_dir}
    RESULT_VARIABLE _doc_snippet_result
    OUTPUT_VARIABLE _doc_snippet_output
    ERROR_VARIABLE _doc_snippet_error)

if (NOT _doc_snippet_result EQUAL 0)
    message(FATAL_ERROR
        "Documentation snippet extraction failed:\n${_doc_snippet_output}${_doc_snippet_error}")
endif ()

message(STATUS "Documentation snippets: ${_doc_snippet_output}")

# Re-run extraction whenever the extractor or any markdown source changes, so an
# incremental build after a doc edit picks up the new snippets.
file(GLOB_RECURSE _doc_snippet_markdown CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.md)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${_doc_snippet_extractor} ${_doc_snippet_markdown})

# Each extractor group compiles in its own object library against the target that
# owns that group's API, and takes its include roots, language level and feature
# macro from that target's interface. Nothing here defines a macro by hand, and no
# object library mixes translation units built under different configurations.
set(_doc_snippet_groups core argmin nlopt urdf)
set(_doc_snippet_core_target cartan::cartan)
set(_doc_snippet_argmin_target cartan::argmin)
set(_doc_snippet_nlopt_target cartan::nlopt)
set(_doc_snippet_urdf_target cartan::urdf)

# The name the rest of the tree builds. It aggregates whichever group libraries
# this configuration created, so a group that compiles is a group that is built.
add_custom_target(doc_snippets)

set(_doc_snippet_compiled 0)

foreach (_group IN LISTS _doc_snippet_groups)
    file(GLOB _group_sources CONFIGURE_DEPENDS ${_doc_snippet_gen_dir}/${_group}/*.cpp)
    list(LENGTH _group_sources _group_count)
    if (_group_count EQUAL 0)
        continue ()
    endif ()

    string(TOUPPER ${_group} _group_upper)
    if (NOT _group STREQUAL core)
        # The extractor emits every marked snippet regardless of the build
        # configuration; a gated group is compiled only when its option is on.
        if (NOT CARTAN_BUILD_${_group_upper})
            message(STATUS "Documentation snippets: skipping ${_group_count} "
                "'${_group}' snippet(s) (CARTAN_BUILD_${_group_upper}=OFF)")
            continue ()
        endif ()
        message(STATUS "Documentation snippets: compiling ${_group_count} "
            "'${_group}' snippet(s) (CARTAN_BUILD_${_group_upper}=ON)")
    endif ()

    add_library(doc_snippets_${_group} OBJECT ${_group_sources})
    target_link_libraries(doc_snippets_${_group} PRIVATE ${_doc_snippet_${_group}_target})
    add_dependencies(doc_snippets doc_snippets_${_group})
    math(EXPR _doc_snippet_compiled "${_doc_snippet_compiled} + ${_group_count}")
endforeach ()

if (_doc_snippet_compiled EQUAL 0)
    message(FATAL_ERROR "Documentation snippet gate is enabled but no snippets were extracted.")
endif ()
