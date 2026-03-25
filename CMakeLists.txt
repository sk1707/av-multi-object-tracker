cmake_minimum_required(VERSION 3.14)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(kalman_filter
    src/main.cpp
    src/ekf.cpp
    src/multi_object_tracker.cpp
)

target_link_libraries(kalman_filter Eigen3::Eigen)

# Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(run_tests tests/test_ekf.cpp src/ekf.cpp src/multi_object_tracker.cpp)
target_link_libraries(run_tests gtest_main Eigen3::Eigen)
add_test(NAME EKFTests COMMAND run_tests)

