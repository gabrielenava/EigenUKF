# Author: Gabriele Nava
# Date: Ott. 2023
cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 11)

# Compile example and test
option(BUILD_EXAMPLE_UKF "Build an example of usage of the UKF code." TRUE)
option(BUILD_TEST_UKF "Build a simple test to inspect library integrity." FALSE)

# Use the variable PROJECT_NAME for changing the target name
get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
set( PROJECT_NAME ${ProjectId} )
project(${PROJECT_NAME})

# Dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Configure and install the library
add_subdirectory(lib)

# Compile the UKF example and test
if(BUILD_EXAMPLE_UKF)
    add_subdirectory(example)
endif()
if(BUILD_TEST_UKF)
    enable_testing()
    add_subdirectory(test)
endif()
