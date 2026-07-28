cmake_minimum_required(VERSION 3.28)
project(cartan_embedded_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CARTAN_SOURCE_DIR "" CACHE PATH "Source tree of the cartan checkout to embed")
if (NOT IS_DIRECTORY "${CARTAN_SOURCE_DIR}")
    message(FATAL_ERROR "CARTAN_SOURCE_DIR must name a cartan source tree, got '${CARTAN_SOURCE_DIR}'")
endif ()

set(parent_shared_before "${BUILD_SHARED_LIBS}")

include(FetchContent)
FetchContent_Declare(cartan SOURCE_DIR "${CARTAN_SOURCE_DIR}")
FetchContent_MakeAvailable(cartan)

if (NOT "${BUILD_SHARED_LIBS}" STREQUAL "${parent_shared_before}")
    message(FATAL_ERROR
        "embedding cartan rewrote this project's BUILD_SHARED_LIBS from "
        "'${parent_shared_before}' to '${BUILD_SHARED_LIBS}'")
endif ()

add_library(parent_default parent.cpp)

get_target_property(parent_type parent_default TYPE)
if (BUILD_SHARED_LIBS AND NOT parent_type STREQUAL "SHARED_LIBRARY")
    message(FATAL_ERROR
        "a library declared with no explicit STATIC/SHARED keyword became ${parent_type} while "
        "this project asked for shared libraries; embedding cartan mutated the default library type")
endif ()

add_executable(embedded_consumer main.cpp)
target_link_libraries(embedded_consumer PRIVATE cartan::cartan)

foreach (backend IN ITEMS argmin)
    string(TOUPPER "${backend}" backend_upper)
    if (CARTAN_BUILD_${backend_upper})
        target_link_libraries(embedded_consumer PRIVATE cartan::${backend})
        target_compile_definitions(embedded_consumer PRIVATE CONSUMER_EXPECTS_${backend_upper})
    endif ()
endforeach ()

enable_testing()
add_test(NAME embedded_consumer COMMAND embedded_consumer)
