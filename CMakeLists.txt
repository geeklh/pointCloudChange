cmake_minimum_required(VERSION 3.21)
project(pointCloudChange)

set(CMAKE_CXX_STANDARD 14)

#pcl
set( CMAKE_PREFIX_PATH  ${} "/usr/local/Cellar/qt/6.3.1_1/lib/cmake/Qt6" )
find_package(PCL 1.12 REQUIRED COMPONENTS common io)
include_directories( ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#vtk

add_executable(pointCloudChange main.cpp)
target_link_libraries(pointCloudChange ${PCL_LIBRARIES})

