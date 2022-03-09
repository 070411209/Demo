cmake_minimum_required(VERSION 2.9)
project(test_object)
set(CMAKE_CXX_STANDARD 11)

option(TEST_DEBUG "option for debug" OFF)
if (TEST_DEBUG)
    add_definitions(-DTEST_DEBUG)
    message(STATUS "ADD TEST_DEBUG")
endif()

add_executable(test_object main.cpp)

