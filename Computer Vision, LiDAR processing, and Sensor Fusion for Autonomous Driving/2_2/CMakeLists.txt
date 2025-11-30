cmake_minimum_required(VERSION 2.8)
project(pose_viewer)
set(CMAKE_CXX_STANDARD 17)

find_package(Pangolin REQUIRED)
if (${Pangolin_FOUND})
    message(STATUS "Found Pangolin - ${Pangolin_DIR}")

    include_directories(${Pangolin_INCLUDE_DIRS})
    set(PANGOLIN_LIBS ${Pangolin_LIBRARIES})

else (${Pangolin_FOUND})
    message(STATUS "Could not support Pangolin")
endif (${Pangolin_FOUND})

find_package(Eigen3 REQUIRED)

# Executable
add_executable(eigen examples/eigen.cpp)
target_link_libraries(eigen Eigen3::Eigen)

add_executable(rotations examples/rotations.cpp)
target_link_libraries(rotations Eigen3::Eigen)

add_executable(pose_viewer examples/pose_viewer.cpp)
target_link_libraries(pose_viewer ${Pangolin_LIBRARIES})
