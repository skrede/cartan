# Reads the kinematics headers and fails when a public entry point in the
# forward-kinematics, Jacobian or velocity family is declared with a return type
# that is not an `expected` and whose name does not end in `_unchecked`. A test
# can only cover the entry points someone remembered to list, so a newly added
# overload is exactly what a test cannot see; this reads the declarations.
#
# HEADERS is a ;-list of absolute paths.

if (NOT DEFINED HEADERS)
    message(FATAL_ERROR "entry_point_gate.cmake requires -DHEADERS")
endif ()

set(entry_point_family
    forward_kinematics
    forward_kinematics_matrix
    space_jacobian
    body_jacobian
    end_effector_velocity)

set(findings "")
set(examined 0)

foreach (header IN LISTS HEADERS)
    if (NOT EXISTS "${header}")
        message(FATAL_ERROR "entry_point_gate.cmake: no such header ${header}")
    endif ()

    # file(STRINGS) would split each C++ statement at its semicolon, so the
    # content is escaped first and then split on newlines only.
    file(READ "${header}" content)
    string(REPLACE ";" "\\;" content "${content}")
    string(REPLACE "\n" ";" lines "${content}")

    get_filename_component(header_name "${header}" NAME)
    set(previous "")
    set(found_here 0)

    foreach (line IN LISTS lines)
        if (line MATCHES "^[ \t]*(///|//|\\*|#)")
            continue ()
        endif ()

        foreach (name IN LISTS entry_point_family)
            set(return_type "")
            set(is_declaration FALSE)
            if (line MATCHES "^[ \t]*${name}[ \t]*\\(")
                set(return_type "${previous}")
                set(is_declaration TRUE)
            elseif (line MATCHES "^([ \t]*[A-Za-z_][^=]*[ \t>])${name}[ \t]*\\(")
                set(return_type "${CMAKE_MATCH_1}")
                set(is_declaration TRUE)
            endif ()

            if (is_declaration)
                math(EXPR found_here "${found_here} + 1")
                math(EXPR examined "${examined} + 1")
                if (NOT return_type MATCHES "expected")
                    list(APPEND findings
                        "${header_name}: ${name} returns '${return_type}'")
                endif ()
            endif ()
        endforeach ()

        set(previous "${line}")
    endforeach ()

    if (found_here EQUAL 0)
        message(FATAL_ERROR
            "entry_point_gate.cmake found no entry point in ${header_name}; "
            "the scan matches nothing and would pass on any content")
    endif ()
endforeach ()

if (findings)
    string(REPLACE ";" "\n  " report "${findings}")
    message(FATAL_ERROR
        "a public kinematics entry point is neither checked nor named _unchecked:\n"
        "  ${report}\n"
        "Give it a wrapper returning cartan::expected, or name it _unchecked.")
endif ()

message(STATUS "Kinematics entry-point gate: ${examined} checked entry points")
