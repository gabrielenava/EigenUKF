# Author: Gabriele Nava
# Date: Ott. 2023

cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 11)

# Use the variable PROJECT_NAME for changing the target name
get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
set( PROJECT_NAME ${ProjectId} )
project(${PROJECT_NAME})

# Dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Configure the library
add_library(${PROJECT_NAME}_lib src/UnscentedKF.cpp src/SystemModel.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC include)

# Configure the executable
set(SRCS src/testUKF.cpp)
add_executable(${PROJECT_NAME} ${SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

# Copy the example data in the build folder
configure_file("${CMAKE_SOURCE_DIR}/data/example_data.txt" "${CMAKE_BINARY_DIR}/example_data.txt" COPYONLY)
