function(recipe_emit out_directory)
    set(emitted "${RECIPE_OUT}/emitted")
    execute_process(
        COMMAND "${RECIPE_PYTHON}" tools/extract_doc_snippets.py
        --docs-root docs --extra-file README.md --require-classified --out "${emitted}"
        WORKING_DIRECTORY "${RECIPE_REPO_ROOT}"
        RESULT_VARIABLE status
        COMMAND_ECHO STDOUT
    )
    if (NOT status EQUAL 0)
        message(FATAL_ERROR "consumability: the snippet extractor failed (exit ${status})")
    endif ()
    if (NOT EXISTS "${emitted}/manifest.txt")
        message(FATAL_ERROR "consumability: the extractor wrote no manifest under ${emitted}")
    endif ()
    set(${out_directory} "${emitted}" PARENT_SCOPE)
endfunction()

# A recipe the manifest names but did not emit, or emitted empty, is the silent
# pass this harness exists to refuse: the consumer would build a body of nothing
# and report success.
function(recipe_body_of emitted line out_label out_path)
    string(REPLACE "\t" ";" fields "${line}")
    list(GET fields 2 name)
    list(GET fields 3 relative)
    set(path "${emitted}/${relative}")
    if (NOT EXISTS "${path}")
        message(FATAL_ERROR "consumability: the manifest names recipe '${name}' at ${relative}, which the extractor did not emit")
    endif ()
    file(READ "${path}" body)
    string(STRIP "${body}" body)
    if (body STREQUAL "")
        message(FATAL_ERROR "consumability: the emitted recipe '${name}' at ${relative} is empty")
    endif ()
    get_filename_component(label "${path}" NAME_WE)
    set(${out_label} "${label}" PARENT_SCOPE)
    set(${out_path} "${path}" PARENT_SCOPE)
endfunction()

function(recipe_select out_var)
    recipe_emit(emitted)
    file(STRINGS "${emitted}/manifest.txt" lines)
    set(selected "")
    foreach (line IN LISTS lines)
        string(REPLACE "\t" ";" fields "${line}")
        list(GET fields 0 kind)
        if (NOT kind STREQUAL "recipe")
            continue ()
        endif ()
        recipe_body_of("${emitted}" "${line}" label path)
        file(READ "${path}" body)
        string(REPLACE "\t" " " body "${body}")
        if (body MATCHES "FetchContent_MakeAvailable *\\( *cartan *\\)")
            list(APPEND selected "${label}=${path}")
        endif ()
    endforeach ()
    if (NOT selected)
        message(FATAL_ERROR "consumability: the manifest carries no build-system recipe that fetches cartan; the published fences lost their sentinels")
    endif ()
    set(${out_var} "${selected}" PARENT_SCOPE)
endfunction()

# The working-tree mode redirects the fetch source, so it never reads the URL or
# the reference the recipe publishes. Resolving them is the half of that gap a
# single network call closes; building from them is what published mode does.
function(recipe_assert_coordinates label body_path)
    file(READ "${body_path}" body)
    string(REPLACE "\t" " " body "${body}")
    if (NOT body MATCHES "GIT_REPOSITORY +([^ \r\n]+)")
        message(FATAL_ERROR "consumability: recipe '${label}' names no GIT_REPOSITORY")
    endif ()
    set(url "${CMAKE_MATCH_1}")
    if (NOT body MATCHES "GIT_TAG +([^ \r\n]+)")
        message(FATAL_ERROR "consumability: recipe '${label}' names no GIT_TAG")
    endif ()
    set(reference "${CMAKE_MATCH_1}")
    execute_process(
        COMMAND "${RECIPE_GIT}" ls-remote --exit-code "${url}" "${reference}"
        RESULT_VARIABLE status
        TIMEOUT 120
        COMMAND_ECHO STDOUT
    )
    if (NOT status EQUAL 0)
        message(FATAL_ERROR "consumability: recipe '${label}' publishes coordinates that do not resolve: ${url} ${reference} (exit ${status})")
    endif ()
endfunction()
