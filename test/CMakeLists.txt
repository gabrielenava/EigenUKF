# Author: Gabriele Nava
# Date: Ott. 2023

include(FetchContent)
FetchContent_Declare(Catch2
                     GIT_REPOSITORY https://github.com/catchorg/Catch2.git
                     GIT_TAG        v3.3.2)

FetchContent_MakeAvailable(Catch2)

include(CTest)

# Add UKF test
set(TEST_TARGET_NAME test_UKF)

add_executable(${TEST_TARGET_NAME} testUKF.cpp)
target_include_directories(${TEST_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${TEST_TARGET_NAME} PRIVATE Catch2::Catch2WithMain ${PROJECT_NAME})
add_test(NAME ${TEST_TARGET_NAME} COMMAND ${TEST_TARGET_NAME})
