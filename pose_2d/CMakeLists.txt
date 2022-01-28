cmake_minimum_required(VERSION 2.9)
project(test_glog)
set(CMAKE_CXX_STANDARD 14)

find_package(Threads)
find_package(gflags)
find_package(glog)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test test.cpp)
target_link_libraries(test Ceres::ceres gflags ${OpenCV_LIBS} )

