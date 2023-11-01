cmake_minimum_required(VERSION 2.8)
project(sophus)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})

# Executable
add_executable(sophus examples/sophus_demo.cpp)
target_link_libraries(sophus ${Eigen3_LIBRARIES} ${Sophus_LIBRARIES})
