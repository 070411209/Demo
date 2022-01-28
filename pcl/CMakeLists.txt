cmake_minimum_required(VERSION 2.4.6)

project(demo)
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")
# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(demo main.cpp)

target_link_libraries(
    demo ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES} 
    fmt
)

add_executable(pcl_demo point_pcl.cpp)

target_link_libraries(
    pcl_demo ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES} 
    fmt
)


add_executable(ndt ndt.cpp)

target_link_libraries(
    ndt 
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES} 
    fmt
)


add_executable(ply save_ply.cpp)

target_link_libraries(
    ply 
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES} 
    fmt
)
