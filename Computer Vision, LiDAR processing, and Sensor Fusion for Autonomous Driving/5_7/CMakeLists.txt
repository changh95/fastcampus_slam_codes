cmake_minimum_required(VERSION 3.10)
project("triangulation" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(Pangolin REQUIRED)
if (${Pangolin_FOUND})
    message(STATUS "Found Pangolin - ${Pangolin_DIR}")

    include_directories(${Pangolin_INCLUDE_DIRS})
    set(PANGOLIN_LIBS ${Pangolin_LIBRARIES})

else (${Pangolin_FOUND})
    message(STATUS "Could not support Pangolin")
endif (${Pangolin_FOUND})

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV library: " ${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (OpenCV_FOUND)

add_executable(triangulation examples/triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS} ${Pangolin_LIBRARIES})