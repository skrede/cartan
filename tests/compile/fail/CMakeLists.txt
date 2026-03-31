cmake_minimum_required(VERSION 3.28)
project(compile_fail_test CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(compile_fail_target ${TEST_SOURCE})
target_include_directories(compile_fail_target SYSTEM PRIVATE
    "${LIEPP_INCLUDE_DIR}"
    "${EIGEN_INCLUDE_DIR}"
)
