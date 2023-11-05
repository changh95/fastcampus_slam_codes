cmake_minimum_required(VERSION 3.10)
project("Introduction to point cloud processing" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
if (PCL_FOUND)
    message(STATUS "Found PCL library: " ${PCL_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
endif (PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(passthrough examples/passthrough.cpp)
target_link_libraries(passthrough ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(kdtree examples/kdtree.cpp)
target_link_libraries(kdtree ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(downsampling examples/downsampling.cpp)
target_link_libraries(downsampling ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(plane_det examples/plane_det.cpp)
target_link_libraries(plane_det ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(sor examples/sor.cpp)
target_link_libraries(sor ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})