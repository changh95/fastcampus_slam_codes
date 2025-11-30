cmake_minimum_required(VERSION 3.10)
project("Iterative Closest Point (ICP)")
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
if (PCL_FOUND)
    message(STATUS "Found PCL library: " ${PCL_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
endif (PCL_FOUND)

find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(OpenMP)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(gicp examples/gicp.cpp)
target_link_libraries(gicp ${PCL_LIBRARIES})

add_executable(ndt examples/ndt.cpp)
target_link_libraries(ndt ${PCL_LIBRARIES})

add_executable(teaser_plus_plus examples/teaser_plus_plus.cpp)
target_link_libraries(teaser_plus_plus Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io OpenMP::OpenMP_CXX ${PCL_LIBRARIES})

add_executable(teaser_plus_plus_kitti examples/teaser_plus_plus_kitti.cpp)
target_link_libraries(teaser_plus_plus_kitti Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io OpenMP::OpenMP_CXX ${PCL_LIBRARIES})
