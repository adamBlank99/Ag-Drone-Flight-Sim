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

add_executable(drone_survey)

target_sources(
    drone_survey
    PRIVATE
        cpp/src/CoveragePlanner.cpp
        cpp/src/main.cpp
)

target_include_directories(
    drone_survey
    PRIVATE
        cpp/include
)

target_compile_features(drone_survey PRIVATE cxx_std_17)
set_target_properties(drone_survey PROPERTIES CXX_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(drone_survey PRIVATE /W4)
else()
    target_compile_options(drone_survey PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
