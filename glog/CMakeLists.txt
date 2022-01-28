cmake_minimum_required(VERSION 2.9)
project(test_glog)
set(CMAKE_CXX_STANDARD 14)

find_package(Threads)
find_package(gflags)
find_package(glog)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

# NLOPT is frustratingly inconsistent in the name of its cmake file so we use our own
list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR})
find_package(NLOPT REQUIRED)

include_directories(
  include
  ${NLOPT_INCLUDE_DIRS}
)

set(LIBS
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
)

add_executable(test_glog main.cpp)
target_link_libraries(test_glog glog gflags ${CMAKE_THREAD_LIBS_INIT} ${NLOPT_LIBRARIES})

add_executable(rotation rotation.cpp)
