cmake_minimum_required(VERSION 3.16)

project(
    AgDroneFlightSim
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not supported. Use: cmake -S . -B build")
endif()

function(enable_project_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(drone_core)

target_sources(
    drone_core
    PRIVATE
        cpp/src/CoveragePlanner.cpp
        cpp/src/Geometry.cpp
        cpp/src/RouteStatistics.cpp
)

target_include_directories(
    drone_core
    PUBLIC
        cpp/include
)

target_compile_features(drone_core PUBLIC cxx_std_17)
set_target_properties(drone_core PROPERTIES CXX_EXTENSIONS OFF)
enable_project_warnings(drone_core)

add_executable(drone_survey cpp/src/main.cpp)
target_link_libraries(drone_survey PRIVATE drone_core)
set_target_properties(drone_survey PROPERTIES CXX_EXTENSIONS OFF)
enable_project_warnings(drone_survey)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
