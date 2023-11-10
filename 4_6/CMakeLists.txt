cmake_minimum_required(VERSION 3.10)
project("Iterative Closest Point (ICP)" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
if (PCL_FOUND)
    message(STATUS "Found PCL library: " ${PCL_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
endif (PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(known_corr_icp examples/known_corr_icp.cpp)
target_link_libraries(known_corr_icp ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(pcl_icp examples/pcl_icp.cpp)
target_link_libraries(pcl_icp ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})