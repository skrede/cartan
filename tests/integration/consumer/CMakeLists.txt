cmake_minimum_required(VERSION 3.28)
project(cartan_integration_test)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(cartan CONFIG REQUIRED)

add_executable(integration_test main.cpp)
target_link_libraries(integration_test PRIVATE cartan::cartan)

enable_testing()
add_test(NAME integration COMMAND integration_test)
