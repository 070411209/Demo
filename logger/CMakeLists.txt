cmake_minimum_required(VERSION 2.9)
project(mlog)
set(CMAKE_CXX_STANDARD 11)

add_executable(mlog main.cpp logger.cpp)
target_link_libraries(mlog "")
