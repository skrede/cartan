# Control for entry_point_gate.cmake. A gate whose scan silently stops matching
# reports every header clean, so the assertion that it still rejects has to be a
# test rather than a one-off run at the time it was written.
#
# Two regressions are exercised against a scratch copy of a real header: an
# overload returning a bare matrix type, and the likelier one — a wrapper with
# the right return type whose body drops the predicate line.

foreach (required GATE_SCRIPT SOURCE_HEADER SCRATCH_DIR EXPECTED)
    if (NOT DEFINED ${required})
        message(FATAL_ERROR "expect_gate_rejects.cmake requires -D${required}")
    endif ()
endforeach ()

file(MAKE_DIRECTORY "${SCRATCH_DIR}")
file(READ "${SOURCE_HEADER}" original)
string(FIND "${original}" "\n}\n\n#endif" tail_position REVERSE)
if (tail_position EQUAL -1)
    message(FATAL_ERROR "expect_gate_rejects.cmake: ${SOURCE_HEADER} has no namespace tail to append to")
endif ()
string(SUBSTRING "${original}" 0 ${tail_position} body)

# Runs the gate over one mutated copy and requires it to fail for the stated
# reason. A control that accepts any nonzero exit also accepts a gate that
# crashed on its own arguments.
function(cartan_expect_gate_rejects label overload want_diagnostic)
    # One directory per case: the gate sweeps its search roots for further
    # headers, and a sibling scratch copy would be a finding of its own.
    set(case_dir "${SCRATCH_DIR}/${label}")
    file(REMOVE_RECURSE "${case_dir}")
    file(MAKE_DIRECTORY "${case_dir}")
    set(scratch "${case_dir}/mutated.h")
    file(WRITE "${scratch}" "${body}\n${overload}\n}\n\n#endif\n")
    math(EXPR mutated_count "${EXPECTED} + 1")

    execute_process(
        COMMAND "${CMAKE_COMMAND}"
                "-DHEADERS=${scratch}" "-DEXPECTED=${mutated_count}"
                "-DSEARCH_ROOTS=${case_dir}"
                -P "${GATE_SCRIPT}"
        OUTPUT_VARIABLE gate_stdout
        ERROR_VARIABLE gate_stderr
        RESULT_VARIABLE gate_result)

    set(report "${gate_stdout}${gate_stderr}")
    if (gate_result EQUAL 0)
        message(FATAL_ERROR "the gate accepted ${label}\n${report}")
    endif ()
    string(FIND "${report}" "${want_diagnostic}" diagnostic_position)
    if (diagnostic_position EQUAL -1)
        message(FATAL_ERROR
            "the gate rejected ${label} for some reason other than '${want_diagnostic}'\n${report}")
    endif ()
endfunction()

cartan_expect_gate_rejects(bare_return
    "template <typename Scalar, int N>
jacobian_matrix<Scalar, N> space_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    int index)
{
    return jacobian_matrix<Scalar, N>{};
}"
    "space_jacobian returns")

cartan_expect_gate_rejects(dropped_predicate
    "template <typename Scalar, int N>
cartan::expected<jacobian_matrix<Scalar, N>, chain_failure> space_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    int index)
{
    return space_jacobian_unchecked(chain, index);
}"
    "is checked in name only")

# The unmutated copy must pass, or the two rejections above could be about the
# copy rather than about the overload appended to it.
file(REMOVE_RECURSE "${SCRATCH_DIR}/clean")
file(MAKE_DIRECTORY "${SCRATCH_DIR}/clean")
set(clean "${SCRATCH_DIR}/clean/unmutated.h")
file(WRITE "${clean}" "${body}\n}\n\n#endif\n")
execute_process(
    COMMAND "${CMAKE_COMMAND}"
            "-DHEADERS=${clean}" "-DEXPECTED=${EXPECTED}"
            "-DSEARCH_ROOTS=${SCRATCH_DIR}/clean"
            -P "${GATE_SCRIPT}"
    OUTPUT_VARIABLE clean_stdout
    ERROR_VARIABLE clean_stderr
    RESULT_VARIABLE clean_result)
if (NOT clean_result EQUAL 0)
    message(FATAL_ERROR
        "the gate rejected an unmutated copy\n${clean_stdout}${clean_stderr}")
endif ()
