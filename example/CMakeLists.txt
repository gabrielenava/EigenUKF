# Author: Gabriele Nava
# Date: Ott. 2023

# Configure the executable
set(SRCS src/ThrustEstimationUKF.cpp src/JetSystemModel.cpp)
add_executable(${PROJECT_NAME}_example ${SRCS})

target_include_directories(${PROJECT_NAME}_example PUBLIC include)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})

# Copy the example data in the build folder
configure_file("${CMAKE_SOURCE_DIR}/example/data/jetData.txt" "${CMAKE_BINARY_DIR}/example/jetData.txt" COPYONLY)
