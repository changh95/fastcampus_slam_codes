cmake_minimum_required(VERSION 3.10)
project("Octree and octomap")
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
if (PCL_FOUND)
    message(STATUS "Found PCL library: " ${PCL_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
endif (PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(octree examples/octree.cpp)
target_link_libraries(octree ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(octomap_kitti examples/octomap_kitti.cpp)
target_link_libraries(octomap_kitti ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${OCTOMAP_LIBRARIES})
