# Reads the public headers and fails when an entry point in the
# forward-kinematics, Jacobian or velocity family is declared with a return type
# that is not an `expected` and whose name does not end in `_unchecked`, or when
# a checked declaration's body calls no shape predicate.
#
# What it does not establish: that the predicate is the *right* one, or that it
# guards every argument. It matches a declaration's return type and looks for a
# predicate call in the lines that follow, so a wrapper calling the wrong
# predicate, or checking one argument of two, passes. That residual is review's
# to carry, and the gate says so in its own output.
#
# HEADERS is a ;-list of absolute paths whose entry points are counted; EXPECTED
# is a parallel ;-list of counts. SEARCH_ROOTS is a ;-list of directories swept
# for any further header declaring an entry point, so a new header is a finding
# rather than an omission.

foreach (required HEADERS EXPECTED SEARCH_ROOTS)
    if (NOT DEFINED ${required})
        message(FATAL_ERROR "entry_point_gate.cmake requires -D${required}")
    endif ()
endforeach ()

set(entry_point_family
    forward_kinematics
    forward_kinematics_matrix
    space_jacobian
    body_jacobian
    end_effector_velocity)

# Scans one header, reports how many checked entry points it declares, and
# appends any finding to `findings` in the caller's scope.
function(cartan_scan_header header found_out)
    file(READ "${header}" content)
    # file(STRINGS) would split each C++ statement at its semicolon, and a
    # declaration may put the name and its parenthesis on separate lines, so the
    # content is escaped, folded and only then split on newlines.
    string(REPLACE ";" "\\;" content "${content}")
    string(REGEX REPLACE "\n[ \t]*\\(" "(" content "${content}")
    string(REPLACE "\n" ";" lines "${content}")

    get_filename_component(header_name "${header}" NAME)
    set(previous "")
    set(found 0)
    set(pending_name "")
    set(pending_line 0)
    set(line_number 0)

    foreach (line IN LISTS lines)
        math(EXPR line_number "${line_number} + 1")

        if (NOT pending_name STREQUAL "")
            if (line MATCHES "check_[a-z_]+\\(")
                set(pending_name "")
            elseif (line MATCHES "^\\}")
                list(APPEND findings
                    "${header_name}:${pending_line}: ${pending_name} is checked in name only, its body calls no predicate")
                set(pending_name "")
            endif ()
        endif ()

        if (line MATCHES "^[ \t]*(///|//|\\*|#)" OR line MATCHES "^[ \t]*return[ \t]")
            set(previous "${line}")
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
                math(EXPR found "${found} + 1")
                if (return_type MATCHES "expected")
                    set(pending_name "${name}")
                    set(pending_line "${line_number}")
                else ()
                    list(APPEND findings
                        "${header_name}:${line_number}: ${name} returns '${return_type}'")
                endif ()
            endif ()
        endforeach ()

        set(previous "${line}")
    endforeach ()

    set(findings "${findings}" PARENT_SCOPE)
    set(${found_out} "${found}" PARENT_SCOPE)
endfunction()

set(findings "")
set(examined 0)
set(index 0)

foreach (header IN LISTS HEADERS)
    if (NOT EXISTS "${header}")
        message(FATAL_ERROR "entry_point_gate.cmake: no such header ${header}")
    endif ()
    list(GET EXPECTED ${index} want)
    math(EXPR index "${index} + 1")

    cartan_scan_header("${header}" found)
    math(EXPR examined "${examined} + ${found}")

    get_filename_component(header_name "${header}" NAME)
    if (NOT found EQUAL want)
        list(APPEND findings
            "${header_name}: declares ${found} entry points, expected ${want}")
    endif ()
endforeach ()

# Any other header declaring an entry point is one the gate was never pointed
# at, which is the omission a hard-coded file list invites.
set(swept "")
foreach (root IN LISTS SEARCH_ROOTS)
    file(GLOB_RECURSE root_headers "${root}/*.h")
    list(APPEND swept ${root_headers})
endforeach ()
if (swept)
    list(REMOVE_ITEM swept ${HEADERS})
endif ()

foreach (header IN LISTS swept)
    cartan_scan_header("${header}" found)
    if (found GREATER 0)
        list(APPEND findings
            "${header} declares ${found} entry points but is not in the gate's header list")
    endif ()
endforeach ()

if (findings)
    string(REPLACE ";" "\n  " report "${findings}")
    message(FATAL_ERROR
        "kinematics entry-point gate:\n  ${report}\n"
        "Give the entry point a wrapper returning cartan::expected that calls a "
        "shape predicate, or name it _unchecked; and keep the gate's expected "
        "counts in tests/boundary/CMakeLists.txt in step.")
endif ()

message(STATUS
    "Kinematics entry-point gate: ${examined} checked entry points. "
    "It matches declaration shape and the presence of a predicate call, not that "
    "the predicate is the right one or that it covers every argument.")
