# Author: Gabriele Nava
# Date: Ott. 2023

# Configure and install the library
add_library(${PROJECT_NAME} src/UnscentedKF.cpp src/UKFModel.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
