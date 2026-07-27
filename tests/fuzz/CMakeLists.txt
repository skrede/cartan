if (NOT CARTAN_BUILD_URDF)
    message(FATAL_ERROR
        "Fuzz targets require the URDF module: fuzz_urdf_parse and fuzz_urdf_build "
        "cover the only boundary in the library where a byte stream from outside "
        "the process is parsed. Building the remaining targets without them would "
        "produce a fuzz configuration that reports success over the boundary that "
        "matters, so the whole block refuses rather than skipping two targets. "
        "Reconfigure with -DCARTAN_BUILD_URDF=ON.")
endif ()

set(cartan_fuzz_binary_dir ${CMAKE_BINARY_DIR}/fuzz)
set(cartan_fuzz_corpus_dir ${cartan_fuzz_binary_dir}/corpus)

# The optimization and debug levels are pinned on the targets rather than left
# to the build type, and both changes are load-bearing. Without -g a sanitizer
# report names no cartan source file and a finding is unattributable. At the
# -O0 a Debug configure would otherwise supply, throughput drops by more than an
# order of magnitude -- measured on the closed-form target, 1.2k executions per
# second against 27k at -O1 -- which is the difference between a bounded run
# reaching a defect and running out of budget short of it.
set(cartan_fuzz_flags
    -fsanitize=fuzzer,address,undefined
    -fno-omit-frame-pointer
    -O1
    -g
)

function(cartan_add_fuzz_target target)
    add_executable(${target} ${target}.cpp)
    target_link_libraries(${target} PRIVATE cartan::cartan)
    target_compile_options(${target} PRIVATE ${cartan_fuzz_flags} ${CARTAN_WARNING_FLAGS})
    target_link_options(${target} PRIVATE ${cartan_fuzz_flags})
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${cartan_fuzz_binary_dir})
endfunction()

# libFuzzer writes every coverage-increasing input back into the directory it is
# given, so it is only ever given this build-tree copy: no run can add to or
# alter the committed fixture set. Only the documents are copied; the licence
# files beside the vendored models are not inputs.
function(cartan_seed_document_corpus target)
    file(GLOB documents
        ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/urdf/*.urdf
        ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/urdf/extended/*.urdf)
    file(COPY ${documents} DESTINATION ${cartan_fuzz_corpus_dir}/${target})
endfunction()

# CMake cannot emit a NUL byte from any command, so a seed that decodes to zero
# is written as the empty file: an exhausted field reads as +0.
function(cartan_write_seed path head_value head_count tail_value tail_count)
    string(ASCII ${head_value} head_byte)
    string(ASCII ${tail_value} tail_byte)
    string(REPEAT "${head_byte}" ${head_count} head)
    string(REPEAT "${tail_byte}" ${tail_count} tail)
    file(WRITE ${path} "${head}${tail}")
endfunction()

# The scalar targets decode fixed-width chunks as IEEE-754 doubles, so their
# seeds are byte patterns rather than documents. A seed only has to put the
# mutator inside the interesting region; it is not a test suite.
function(cartan_seed_scalar_corpus target)
    set(dir ${cartan_fuzz_corpus_dir}/${target})
    file(WRITE ${dir}/zero_fields "")
    cartan_write_seed(${dir}/nonfinite_fields 255 64 255 0)
    cartan_write_seed(${dir}/descending_pair 65 8 49 56)
    cartan_write_seed(${dir}/small_magnitude_fields 1 64 1 0)
    cartan_write_seed(${dir}/large_negative_fields 240 64 240 0)
endfunction()

cartan_add_fuzz_target(fuzz_urdf_parse)
cartan_add_fuzz_target(fuzz_urdf_build)
cartan_add_fuzz_target(fuzz_screw_axis)
cartan_add_fuzz_target(fuzz_joint_limits)
cartan_add_fuzz_target(fuzz_analytical_target)

cartan_seed_document_corpus(fuzz_urdf_parse)
cartan_seed_scalar_corpus(fuzz_urdf_build)
cartan_seed_scalar_corpus(fuzz_screw_axis)
cartan_seed_scalar_corpus(fuzz_joint_limits)
cartan_seed_scalar_corpus(fuzz_analytical_target)
