add_executable(include_test include_test.cpp)
target_link_libraries(include_test PRIVATE cartan::cartan)
target_compile_options(include_test PRIVATE ${CARTAN_WARNING_FLAGS})
add_test(NAME include_test COMMAND include_test)

# The standard is declared by target_compile_features on the cartan targets and
# nowhere else, so linking cartan::cartan is what puts the standard flag on this
# unit's command line. The assertion is an equality: a later standard fails it
# just as an earlier one does, which is what makes it a check on the declaration
# rather than a lower bound.
add_executable(standard_assertion_test standard_assertion_test.cpp)
target_link_libraries(standard_assertion_test PRIVATE cartan::cartan)
target_compile_options(standard_assertion_test PRIVATE ${CARTAN_WARNING_FLAGS})
add_test(NAME standard_assertion_test COMMAND standard_assertion_test)

# Two-mode proof that the validated Lie factories compile with C++ exceptions
# both disabled and enabled. The exceptions-off build mirrors the embedded
# posture (ESP-IDF defaults to -fno-exceptions -fno-rtti) and guards against a
# regression that reintroduces an ungated throw on a factory-reachable path.
# The parity build keeps the exceptions-on behavior honest. Building either
# target is the test: if it compiles and links, the mode is sound.
set(CARTAN_NO_EXCEPTIONS_FLAGS
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fno-exceptions -fno-rtti>)

add_executable(no_exceptions_factory_test no_exceptions_factory_test.cpp)
target_link_libraries(no_exceptions_factory_test PRIVATE cartan::cartan)
target_compile_options(no_exceptions_factory_test
    PRIVATE ${CARTAN_WARNING_FLAGS} ${CARTAN_NO_EXCEPTIONS_FLAGS})
add_test(NAME no_exceptions_factory_test COMMAND no_exceptions_factory_test)

add_executable(exceptions_factory_parity_test no_exceptions_factory_test.cpp)
target_link_libraries(exceptions_factory_parity_test PRIVATE cartan::cartan)
target_compile_options(exceptions_factory_parity_test PRIVATE ${CARTAN_WARNING_FLAGS})
add_test(NAME exceptions_factory_parity_test COMMAND exceptions_factory_parity_test)

# Compile-fail test infrastructure
# Each test should FAIL to compile. If it compiles, that's a test failure.
# Uses try_compile at configure time to verify type-safety errors.
#
# Every cartan include root is mapped, not just the Lie one, so a compile-fail
# test may name any public header. Eigen comes from the imported target rather
# than from eigen3_SOURCE_DIR, which is populated only on the FetchContent
# branch: with a system Eigen it was empty, every one of these tests failed with
# "Eigen/Dense: No such file or directory", and each reported success while
# never reaching the code it exists to reject. Deleting the type safety outright
# would not have failed any of them.
set(CARTAN_COMPILE_FAIL_INCLUDES
    "${PROJECT_SOURCE_DIR}/lib/cartan-lie/include"
    "${PROJECT_SOURCE_DIR}/lib/cartan-serial-chain/include"
    "${PROJECT_SOURCE_DIR}/lib/cartan-analytical/include")

# find_package's imported targets are scoped to the directory that called it,
# and that call lives in lib/cartan-lie, so Eigen3::Eigen is not visible here.
# Re-resolve it locally; the find is cached and costs nothing.
if (NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 CONFIG QUIET)
endif ()
if (TARGET Eigen3::Eigen)
    get_target_property(CARTAN_COMPILE_FAIL_EIGEN Eigen3::Eigen
        INTERFACE_INCLUDE_DIRECTORIES)
endif ()
if (NOT CARTAN_COMPILE_FAIL_EIGEN)
    set(CARTAN_COMPILE_FAIL_EIGEN "${eigen3_SOURCE_DIR}")
endif ()
if (NOT CARTAN_COMPILE_FAIL_EIGEN)
    message(FATAL_ERROR
        "No Eigen include directory for the compile-fail subproject; every test in it would "
        "fail on a missing header and report success.")
endif ()

function(cartan_add_compile_fail_test TEST_NAME TEST_SOURCE)
    string(REPLACE ";" "|" fail_includes "${CARTAN_COMPILE_FAIL_INCLUDES}")
    string(REPLACE ";" "|" eigen_includes "${CARTAN_COMPILE_FAIL_EIGEN}")
    try_compile(${TEST_NAME}_COMPILES
        PROJECT compile_fail_${TEST_NAME}
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/fail"
        BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}_test"
        TARGET compile_fail_target
        CMAKE_FLAGS
            "-DCMAKE_CXX_STANDARD=23"
            "-DCARTAN_INCLUDE_DIR=${fail_includes}"
            "-DEIGEN_INCLUDE_DIR=${eigen_includes}"
            "-DTEST_SOURCE=${TEST_SOURCE}"
        OUTPUT_VARIABLE compile_fail_output
    )

    if(${TEST_NAME}_COMPILES)
        message(FATAL_ERROR "Compile-fail test '${TEST_NAME}' unexpectedly compiled!")
    endif()

    # A missing header fails the compile just as an ill-formed program does, and
    # is the way this gate stops testing anything without saying so.
    if(compile_fail_output MATCHES "No such file or directory|file not found")
        message(FATAL_ERROR
            "Compile-fail test '${TEST_NAME}' failed on a missing header rather than on the "
            "construct it exists to reject; its include plumbing is broken:\n${compile_fail_output}")
    endif()

    add_test(NAME compile_fail_${TEST_NAME}
        COMMAND ${CMAKE_COMMAND} -E echo "PASS: ${TEST_NAME} correctly fails to compile")
endfunction()

# A control is the same subproject and the same include plumbing running a
# well-formed program: without one, a rejection beside it proves nothing, since
# a broken include list rejects everything.
function(cartan_add_compile_pass_test TEST_NAME TEST_SOURCE)
    string(REPLACE ";" "|" pass_includes "${CARTAN_COMPILE_FAIL_INCLUDES}")
    string(REPLACE ";" "|" pass_eigen "${CARTAN_COMPILE_FAIL_EIGEN}")
    try_compile(${TEST_NAME}_COMPILES
        PROJECT compile_pass_${TEST_NAME}
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/fail"
        BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}_control"
        TARGET compile_fail_target
        CMAKE_FLAGS
            "-DCMAKE_CXX_STANDARD=23"
            "-DCARTAN_INCLUDE_DIR=${pass_includes}"
            "-DEIGEN_INCLUDE_DIR=${pass_eigen}"
            "-DTEST_SOURCE=${TEST_SOURCE}"
        OUTPUT_VARIABLE compile_pass_output
    )

    if(NOT ${TEST_NAME}_COMPILES)
        message(FATAL_ERROR
            "The compile-fail control '${TEST_NAME}' did not compile, so the rejection it "
            "controls for proves nothing:\n${compile_pass_output}")
    endif()

    add_test(NAME compile_pass_${TEST_NAME}
        COMMAND ${CMAKE_COMMAND} -E echo "PASS: ${TEST_NAME} compiles as it must")
endfunction()

cartan_add_compile_pass_test(control joint_limits_control.cpp)
cartan_add_compile_pass_test(factory_pieper_constructor factory_pieper_constructor.cpp)

cartan_add_compile_fail_test(frame_mismatch_compose frame_mismatch_compose.cpp)
cartan_add_compile_fail_test(frame_mismatch_inverse_compose frame_mismatch_inverse_compose.cpp)
cartan_add_compile_fail_test(frame_mismatch_adjoint_twist frame_mismatch_adjoint_twist.cpp)
cartan_add_compile_fail_test(frame_mismatch_coadjoint_wrench frame_mismatch_coadjoint_wrench.cpp)
cartan_add_compile_fail_test(joint_limits_aggregate joint_limits_aggregate.cpp)
cartan_add_compile_fail_test(joint_limits_field_assign joint_limits_field_assign.cpp)
cartan_add_compile_fail_test(joint_limits_member_assign joint_limits_member_assign.cpp)
cartan_add_compile_fail_test(private_pieper_constructor private_pieper_constructor.cpp)
