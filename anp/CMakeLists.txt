cmake_minimum_required(VERSION 3.10.0)

project(Demo)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_executable(Demo main.cpp)

target_link_libraries(
    Demo
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

add_executable(calib calibration.cpp)

target_link_libraries(
    calib
    ${OpenCV_LIBS}
)
