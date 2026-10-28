cmake_minimum_required(VERSION 3.28)
project(compile_fail_test CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The include lists arrive '|'-separated: a ';' would be consumed as a list
# separator by the CMAKE_FLAGS the parent hands to try_compile.
string(REPLACE "|" ";" CARTAN_INCLUDE_DIRS "${CARTAN_INCLUDE_DIR}")
string(REPLACE "|" ";" EIGEN_INCLUDE_DIRS "${EIGEN_INCLUDE_DIR}")

add_executable(compile_fail_target ${TEST_SOURCE})
target_include_directories(compile_fail_target SYSTEM PRIVATE
    ${CARTAN_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
