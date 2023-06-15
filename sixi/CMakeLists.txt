cmake_minimum_required(VERSION 3.10.0)

project(Demo)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(demo main.cpp)

target_link_libraries(demo ${PCL_LIBRARIES})
