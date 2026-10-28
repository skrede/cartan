cmake_minimum_required(VERSION 3.28)
project(cartan_installed_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cartan CONFIG REQUIRED)

add_executable(installed_consumer main.cpp)
target_link_libraries(installed_consumer PRIVATE cartan::cartan)

enable_testing()
add_test(NAME installed_consumer COMMAND installed_consumer)
