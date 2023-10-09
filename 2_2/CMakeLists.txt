cmake_minimum_required(VERSION 2.8)
project(pose_viewer)

find_package(Pangolin REQUIRED)
if (${Pangolin_FOUND})
    message(STATUS "Found Pangolin - ${Pangolin_DIR}")

    include_directories(${Pangolin_INCLUDE_DIRS})
    set(PANGOLIN_LIBS ${Pangolin_LIBRARIES})

else (${Pangolin_FOUND})
    message(STATUS "Could not support Pangolin")
endif (${Pangolin_FOUND})

# Executable
add_executable(pose_viewer examples/pose_viewer.cpp)
target_link_libraries(pose_viewer ${Pangolin_LIBRARIES})
