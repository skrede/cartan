cmake_minimum_required(VERSION 3.28)
project(cartan_installed_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CONSUMER_EXPECTS_ARGMIN "Link the argmin component of the installed cartan package" OFF)

if (CONSUMER_EXPECTS_ARGMIN)
    find_package(cartan CONFIG REQUIRED COMPONENTS argmin)
else ()
    find_package(cartan CONFIG REQUIRED)
endif ()

add_executable(installed_consumer main.cpp)
target_link_libraries(installed_consumer PRIVATE cartan::cartan)

if (CONSUMER_EXPECTS_ARGMIN)
    target_link_libraries(installed_consumer PRIVATE cartan::argmin)
    target_compile_definitions(installed_consumer PRIVATE CONSUMER_EXPECTS_ARGMIN)
endif ()

enable_testing()
add_test(NAME installed_consumer COMMAND installed_consumer)
