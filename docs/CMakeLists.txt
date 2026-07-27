# Documentation snippet compile gate.
#
# The guides carry HTML-comment sentinels marking which fenced ``cpp`` blocks are
# real, buildable code. tools/extract_doc_snippets.py turns each marked block into
# a standalone translation unit; those units are compiled here against the actual
# public headers via the cartan::cartan target. A renamed symbol or a wrong
# signature in the prose therefore breaks the build instead of rotting silently.
#
# This target is compile-only: it is never linked into an executable and never
# run. Its sole purpose is to force the compiler across the documented API.

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

# The always-compiled core group: snippets with no backend requirement. Include
# roots and the cxx_std_20 feature come entirely from the linked cartan::cartan
# target -- there is no hand-rolled -I list anywhere in this file.
file(GLOB _doc_snippet_core CONFIGURE_DEPENDS ${_doc_snippet_gen_dir}/core/*.cpp)

# Backend-gated groups. The extractor emits every marked snippet regardless of the
# build configuration; a ``needs=<cap>`` group is compiled only when the matching
# backend option is on, and reported (skipped) otherwise.
file(GLOB _doc_snippet_argmin CONFIGURE_DEPENDS ${_doc_snippet_gen_dir}/argmin/*.cpp)
file(GLOB _doc_snippet_nlopt CONFIGURE_DEPENDS ${_doc_snippet_gen_dir}/nlopt/*.cpp)
file(GLOB _doc_snippet_urdf CONFIGURE_DEPENDS ${_doc_snippet_gen_dir}/urdf/*.cpp)

set(_doc_snippet_sources ${_doc_snippet_core})
set(_doc_snippet_link_libs cartan::cartan)
set(_doc_snippet_argmin_group "")

list(LENGTH _doc_snippet_argmin _doc_snippet_argmin_count)
if (_doc_snippet_argmin_count GREATER 0)
    if (CARTAN_BUILD_ARGMIN)
        set(_doc_snippet_argmin_group ${_doc_snippet_argmin})
        message(STATUS
            "Documentation snippets: compiling ${_doc_snippet_argmin_count} 'argmin' snippet(s) (CARTAN_BUILD_ARGMIN=ON)")
    else ()
        message(STATUS
            "Documentation snippets: skipping ${_doc_snippet_argmin_count} 'argmin' snippet(s) (CARTAN_BUILD_ARGMIN=OFF)")
    endif ()
endif ()

list(LENGTH _doc_snippet_nlopt _doc_snippet_nlopt_count)
if (_doc_snippet_nlopt_count GREATER 0)
    if (CARTAN_BUILD_NLOPT)
        list(APPEND _doc_snippet_sources ${_doc_snippet_nlopt})
        # The NLopt backend carries its include roots and the CARTAN_HAS_NLOPT
        # guard on the separate cartan::nlopt target; link it for that group.
        list(APPEND _doc_snippet_link_libs cartan::nlopt)
        message(STATUS
            "Documentation snippets: compiling ${_doc_snippet_nlopt_count} 'nlopt' snippet(s) (CARTAN_BUILD_NLOPT=ON)")
    else ()
        message(STATUS
            "Documentation snippets: skipping ${_doc_snippet_nlopt_count} 'nlopt' snippet(s) (CARTAN_BUILD_NLOPT=OFF)")
    endif ()
endif ()

list(LENGTH _doc_snippet_urdf _doc_snippet_urdf_count)
if (_doc_snippet_urdf_count GREATER 0)
    if (CARTAN_BUILD_URDF)
        list(APPEND _doc_snippet_sources ${_doc_snippet_urdf})
        list(APPEND _doc_snippet_link_libs cartan::urdf)
        message(STATUS
            "Documentation snippets: compiling ${_doc_snippet_urdf_count} 'urdf' snippet(s) (CARTAN_BUILD_URDF=ON)")
    else ()
        message(STATUS
            "Documentation snippets: skipping ${_doc_snippet_urdf_count} 'urdf' snippet(s) (CARTAN_BUILD_URDF=OFF)")
    endif ()
endif ()

list(LENGTH _doc_snippet_sources _doc_snippet_source_count)
list(LENGTH _doc_snippet_argmin_group _doc_snippet_group_count)
math(EXPR _doc_snippet_source_count "${_doc_snippet_source_count} + ${_doc_snippet_group_count}")
if (_doc_snippet_source_count EQUAL 0)
    message(FATAL_ERROR "Documentation snippet gate is enabled but no snippets were extracted.")
endif ()

add_library(doc_snippets OBJECT ${_doc_snippet_sources})
target_link_libraries(doc_snippets PRIVATE ${_doc_snippet_link_libs})

# The argmin group compiles in its own object library so the core group keeps
# proving that the documented core API needs no backend.
if (_doc_snippet_argmin_group)
    add_library(doc_snippets_argmin OBJECT ${_doc_snippet_argmin_group})
    target_link_libraries(doc_snippets_argmin PRIVATE ${_doc_snippet_link_libs} cartan::argmin)
endif ()
