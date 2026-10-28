include_guard(GLOBAL)

# What a benchmark dependency has to say about itself before it may appear in a
# published table, and how it says it.
#
# A number attributed to "the comparator" and nothing more is a number attributed
# to nothing: the reader cannot tell which code produced it, and neither can the
# next run. So every dependency the build links is recorded with the class it was
# acquired under, the provider that supplied it and the version or commit it
# reports -- and a dependency that is present but cannot say which version it is
# stops the configure rather than being linked unlabelled.

set(CARTAN_BENCH_DEPENDENCY_RECORDS "")
set(CARTAN_BENCH_ABSENT_COMPARATORS "")

# The separators the manifest string is assembled with. A field carrying either
# would split into two, so both are stripped out of every value on the way in.
set(CARTAN_BENCH_FIELD_SEPARATOR "|")
set(CARTAN_BENCH_RECORD_SEPARATOR "@")

function(cartan_bench_sanitize VALUE OUTPUT)
    string(REPLACE "${CARTAN_BENCH_FIELD_SEPARATOR}" " " clean "${VALUE}")
    string(REPLACE "${CARTAN_BENCH_RECORD_SEPARATOR}" " " clean "${clean}")
    string(REPLACE ";" "," clean "${clean}")
    string(REPLACE "\"" "'" clean "${clean}")
    if (DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
        string(REPLACE "$ENV{HOME}" "~" clean "${clean}")
    endif ()
    set(${OUTPUT} "${clean}" PARENT_SCOPE)
endfunction()

# The version a source tree declares about itself, read off its own project()
# call. This is the only mechanism available for a dependency that ships neither
# a package configuration nor a version header.
function(cartan_bench_project_version DIRECTORY OUTPUT)
    set(version "")
    if (EXISTS "${DIRECTORY}/CMakeLists.txt")
        file(STRINGS "${DIRECTORY}/CMakeLists.txt" declaration
            REGEX "[Pp][Rr][Oo][Jj][Ee][Cc][Tt] *\\(.*VERSION +[0-9]" LIMIT_COUNT 1)
        if (declaration)
            string(REGEX MATCH "VERSION +([0-9][0-9a-zA-Z.+-]*)" _matched "${declaration}")
            set(version "${CMAKE_MATCH_1}")
        endif ()
    endif ()
    set(${OUTPUT} "${version}" PARENT_SCOPE)
endfunction()

function(cartan_bench_git_revision DIRECTORY OUTPUT)
    set(revision "")
    find_package(Git QUIET)
    if (GIT_EXECUTABLE AND EXISTS "${DIRECTORY}/.git")
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${DIRECTORY}" rev-parse HEAD
            RESULT_VARIABLE status OUTPUT_VARIABLE head ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        if (status EQUAL 0)
            set(revision "${head}")
        endif ()
    endif ()
    set(${OUTPUT} "${revision}" PARENT_SCOPE)
endfunction()

# The version carried by the versioned name of the shared object the build
# actually resolved to. This is the last mechanism available for a library that
# ships neither a usable package configuration nor a version macro, and it names
# the file that will be loaded rather than the one a search path suggests.
function(cartan_bench_library_version LIBRARIES OUTPUT)
    set(version "")
    foreach (candidate IN LISTS LIBRARIES)
        if (NOT EXISTS "${candidate}")
            continue ()
        endif ()
        get_filename_component(resolved "${candidate}" REALPATH)
        if (resolved MATCHES "\\.(so|dylib)\\.([0-9][0-9.]*)$")
            set(version "${CMAKE_MATCH_2}")
        elseif (resolved MATCHES "\\.([0-9][0-9.]*)\\.dylib$")
            set(version "${CMAKE_MATCH_1}")
        endif ()
        if (NOT version STREQUAL "")
            break ()
        endif ()
    endforeach ()
    set(${OUTPUT} "${version}" PARENT_SCOPE)
endfunction()

function(cartan_bench_refuse_unversioned NAME VERSION MECHANISMS)
    if (VERSION STREQUAL "")
        message(FATAL_ERROR
            "cartan benchmarks: '${NAME}' was found but its version could not be determined. "
            "Tried: ${MECHANISMS}. A comparator that cannot be identified cannot appear in a "
            "published table, so the configure stops here rather than linking it unlabelled.")
    endif ()
endfunction()

# Only the owned class has a pin to mismatch against: this build chooses those
# revisions, so the revision it resolved and the revision it declares must be the
# same code. A previous capture ran fifty-one commits ahead of the pin and
# nothing noticed.
function(cartan_bench_verify_pin NAME DIRECTORY DECLARED)
    cartan_bench_git_revision("${DIRECTORY}" resolved)
    if (resolved STREQUAL "")
        message(FATAL_ERROR
            "cartan benchmarks: '${NAME}' is pinned at ${DECLARED} but the revision of the "
            "checkout at ${DIRECTORY} could not be read, so the pin cannot be verified.")
    endif ()
    if (NOT resolved STREQUAL DECLARED)
        message(FATAL_ERROR
            "cartan benchmarks: '${NAME}' resolved to ${resolved} while the declared pin is "
            "${DECLARED}. The published numbers and the pin must name the same code: either "
            "move the pin, or point the build at the pinned revision.")
    endif ()
endfunction()

macro(cartan_bench_record NAME CLASS PROVIDER VERSION COUNTABLE)
    string(TOUPPER "${NAME}" _record_key)
    cartan_bench_sanitize("${PROVIDER}" _record_provider)
    cartan_bench_sanitize("${VERSION}" _record_version)
    set(CARTAN_BENCH_${_record_key}_FOUND TRUE)
    set(CARTAN_BENCH_${_record_key}_CLASS "${CLASS}")
    set(CARTAN_BENCH_${_record_key}_PROVIDER "${_record_provider}")
    set(CARTAN_BENCH_${_record_key}_VERSION "${_record_version}")
    list(APPEND CARTAN_BENCH_DEPENDENCY_RECORDS
        "${NAME}|${CLASS}|${_record_provider}|${_record_version}|${COUNTABLE}")
    message(STATUS "cartan benchmarks: ${NAME} [${CLASS}] from ${_record_provider}, "
        "version ${_record_version}")
endmacro()

# An absent comparator is the ordinary path under system-provided dependencies,
# not an error path -- but a run that resolved fewer participants than it
# declared reads exactly like a complete one, which is why the omission is
# announced here, again at run start, and again in the manifest.
macro(cartan_bench_absent NAME CLASS REASON)
    string(TOUPPER "${NAME}" _absent_key)
    cartan_bench_sanitize("${REASON}" _absent_reason)
    set(CARTAN_BENCH_${_absent_key}_FOUND FALSE)
    set(CARTAN_BENCH_${_absent_key}_CLASS "${CLASS}")
    set(CARTAN_BENCH_${_absent_key}_PROVIDER "absent")
    set(CARTAN_BENCH_${_absent_key}_VERSION "")
    list(APPEND CARTAN_BENCH_ABSENT_COMPARATORS "${NAME}|${_absent_reason}")
    if (CARTAN_STRICT_COMPARATORS)
        message(FATAL_ERROR
            "cartan benchmarks: '${NAME}' is enabled but was not found -- ${_absent_reason}. "
            "CARTAN_STRICT_COMPARATORS is ON, so the configure refuses a partial participant set.")
    endif ()
    message(STATUS "cartan benchmarks: ${NAME} [${CLASS}] NOT FOUND -- ${_absent_reason}; "
        "its cells are omitted from this build")
endmacro()

function(cartan_bench_join RECORDS OUTPUT)
    string(REPLACE ";" "${CARTAN_BENCH_RECORD_SEPARATOR}" joined "${RECORDS}")
    set(${OUTPUT} "${joined}" PARENT_SCOPE)
endfunction()

# Reconstructed from the cache rather than captured, because CMake does not keep
# the literal command line: running this reproduces this configuration. Paths
# under the developer's home directory are written as ~.
function(cartan_bench_configure_command OUTPUT)
    set(command "cmake -S ${CMAKE_SOURCE_DIR} -B ${CMAKE_BINARY_DIR} -G ${CMAKE_GENERATOR}")
    get_cmake_property(names CACHE_VARIABLES)
    list(SORT names)
    foreach (name IN LISTS names)
        get_property(kind CACHE "${name}" PROPERTY TYPE)
        if (kind STREQUAL "INTERNAL" OR kind STREQUAL "STATIC")
            continue ()
        endif ()
        if (NOT name MATCHES "^(CARTAN_|CMAKE_BUILD_TYPE$|CMAKE_PREFIX_PATH$|CMAKE_CXX_FLAGS$)")
            continue ()
        endif ()
        if ("${${name}}" STREQUAL "")
            continue ()
        endif ()
        string(APPEND command " -D${name}=${${name}}")
    endforeach ()
    cartan_bench_sanitize("${command}" command)
    set(${OUTPUT} "${command}" PARENT_SCOPE)
endfunction()
