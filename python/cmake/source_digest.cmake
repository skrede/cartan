# Hash every source the Python extension is compiled from, and write the result
# to a header the module exports as __source_digest__. The test suite recomputes
# the same digest and refuses to run when it differs, which catches an extension
# built from other sources than the ones on disk -- including an edit under lib/,
# where the behavior the bindings expose actually lives. Content, not mtime: a
# restored artifact cache or a bare touch of the module cannot defeat it.
#
# Run in script mode with CARTAN_DIGEST_ROOT and CARTAN_DIGEST_OUT defined.
# The algorithm is mirrored in python/tests/conftest.py and the two must agree:
# "<relative-path> <sha256-of-file>\n" per file, ordered by relative path, then
# SHA256 over the concatenation.

cmake_minimum_required(VERSION 3.28)

set(_digest_globs
    "${CARTAN_DIGEST_ROOT}/lib/*.h"
    "${CARTAN_DIGEST_ROOT}/lib/*.hpp"
    "${CARTAN_DIGEST_ROOT}/lib/*.hxx"
    "${CARTAN_DIGEST_ROOT}/lib/*.inl"
    "${CARTAN_DIGEST_ROOT}/lib/*.cpp"
    "${CARTAN_DIGEST_ROOT}/lib/*.cxx"
    "${CARTAN_DIGEST_ROOT}/python/src/*.h"
    "${CARTAN_DIGEST_ROOT}/python/src/*.hpp"
    "${CARTAN_DIGEST_ROOT}/python/src/*.hxx"
    "${CARTAN_DIGEST_ROOT}/python/src/*.inl"
    "${CARTAN_DIGEST_ROOT}/python/src/*.cpp"
    "${CARTAN_DIGEST_ROOT}/python/src/*.cxx"
)

file(GLOB_RECURSE _digest_sources
    LIST_DIRECTORIES false
    RELATIVE "${CARTAN_DIGEST_ROOT}"
    ${_digest_globs}
)
list(SORT _digest_sources)

set(_digest_payload "")
foreach (_source IN LISTS _digest_sources)
    file(SHA256 "${CARTAN_DIGEST_ROOT}/${_source}" _source_hash)
    string(APPEND _digest_payload "${_source} ${_source_hash}\n")
endforeach ()

string(SHA256 _digest "${_digest_payload}")
list(LENGTH _digest_sources _digest_count)

file(WRITE "${CARTAN_DIGEST_OUT}.tmp"
    "#ifndef HPP_GUARD_CARTAN_PYTHON_SOURCE_DIGEST_H\n"
    "#define HPP_GUARD_CARTAN_PYTHON_SOURCE_DIGEST_H\n"
    "\n"
    "#define CARTAN_PY_SOURCE_DIGEST \"${_digest}\"\n"
    "#define CARTAN_PY_SOURCE_COUNT ${_digest_count}\n"
    "\n"
    "#endif\n"
)
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different
    "${CARTAN_DIGEST_OUT}.tmp" "${CARTAN_DIGEST_OUT}")
