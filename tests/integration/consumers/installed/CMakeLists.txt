cmake_minimum_required(VERSION 3.28)
project(cartan_installed_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CONSUMER_EXPECTS_ARGMIN "Link the argmin component of the installed cartan package" OFF)
option(CONSUMER_EXPECTS_NLOPT "Link the nlopt component of the installed cartan package" OFF)

set(consumer_components "")
if (CONSUMER_EXPECTS_ARGMIN)
    list(APPEND consumer_components argmin)
endif ()
if (CONSUMER_EXPECTS_NLOPT)
    list(APPEND consumer_components nlopt)
endif ()

if (consumer_components)
    find_package(cartan CONFIG REQUIRED COMPONENTS ${consumer_components})
else ()
    find_package(cartan CONFIG REQUIRED)
endif ()

add_executable(installed_consumer main.cpp)
target_link_libraries(installed_consumer PRIVATE cartan::cartan)

foreach (component IN LISTS consumer_components)
    string(TOUPPER "${component}" component_upper)
    target_link_libraries(installed_consumer PRIVATE cartan::${component})
    target_compile_definitions(installed_consumer PRIVATE CONSUMER_EXPECTS_${component_upper})
endforeach ()

enable_testing()
add_test(NAME installed_consumer COMMAND installed_consumer)
