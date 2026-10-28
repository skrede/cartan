if (NOT CARTAN_BUILD_URDF)
    message(FATAL_ERROR
        "Fuzz targets require the URDF module: fuzz_urdf_parse and fuzz_urdf_build "
        "cover the only boundary in the library where a byte stream from outside "
        "the process is parsed. Building the remaining targets without them would "
        "produce a fuzz configuration that reports success over the boundary that "
        "matters, so the whole block refuses rather than skipping two targets. "
        "Reconfigure with -DCARTAN_BUILD_URDF=ON.")
endif ()

cmake_language(DEFER CALL cartan_warn_directory_targets)

set(cartan_fuzz_binary_dir ${CMAKE_BINARY_DIR}/fuzz)
set(cartan_fuzz_corpus_dir ${cartan_fuzz_binary_dir}/corpus)

# Every flag here is pinned on the targets rather than left to the build type,
# and each one is load-bearing. Without -fno-sanitize-recover an undefined-
# behavior violation prints a diagnostic and execution continues: the fuzzer
# sees a normal return, retains no artifact, and the process exits zero, so the
# gate cannot go red on a real finding. Without -g a report names no cartan
# source file and a finding is unattributable. At the -O0 a Debug configure
# would otherwise supply, throughput drops by more than an order of magnitude,
# which is the difference between a bounded run reaching a defect and running
# out of budget short of it.
set(cartan_fuzz_flags
    -fsanitize=fuzzer,address,undefined
    -fno-sanitize-recover=undefined
    -fno-omit-frame-pointer
    -O1
    -g
)

function(cartan_add_fuzz_target target)
    add_executable(${target} ${target}.cpp)
    target_link_libraries(${target} PRIVATE cartan::cartan)
    target_compile_options(${target} PRIVATE ${cartan_fuzz_flags})
    target_link_options(${target} PRIVATE ${cartan_fuzz_flags})
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${cartan_fuzz_binary_dir})
endfunction()

# A replay is registered so that something in the repository executes these
# targets rather than only building them. Nothing here registers a fuzzing run:
# that is a campaign, it needs a budget, and it belongs to a job rather than to
# ctest.
function(cartan_add_fuzz_replay target minimum)
    add_test(NAME ${target}_replay COMMAND ${CMAKE_COMMAND}
        -DTARGET=$<TARGET_FILE:${target}>
        -DCORPUS=${cartan_fuzz_corpus_dir}/${target}
        -DMINIMUM=${minimum}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/replay.cmake)
    set_tests_properties(${target}_replay PROPERTIES
        ENVIRONMENT "UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1")
endfunction()

# libFuzzer writes every coverage-increasing input back into the directory it is
# given, so it is only ever given this build-tree copy: no run can add to or
# alter the committed fixture set. Only the documents are copied; the license
# files beside the vendored models are not inputs. The copy is additive and
# nothing prunes it, so a directory a long run has grown keeps every input until
# it is merged with -merge=1 or deleted.
function(cartan_seed_document_corpus target)
    file(GLOB documents CONFIGURE_DEPENDS
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

cartan_add_fuzz_replay(fuzz_urdf_parse 20)
cartan_add_fuzz_replay(fuzz_urdf_build 5)
cartan_add_fuzz_replay(fuzz_screw_axis 5)
cartan_add_fuzz_replay(fuzz_joint_limits 5)
cartan_add_fuzz_replay(fuzz_analytical_target 5)
