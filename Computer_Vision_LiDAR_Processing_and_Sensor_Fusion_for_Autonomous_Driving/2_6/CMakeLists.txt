cmake_minimum_required(VERSION 2.8)
project(sophus_demo)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories("/usr/include/eigen3" "/usr/local/include")

# Executable
add_executable(sophus_demo examples/sophus_demo.cpp)
target_link_libraries(sophus_demo Eigen3::Eigen Sophus::Sophus)
