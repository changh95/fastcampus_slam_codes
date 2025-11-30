cmake_minimum_required(VERSION 3.10)
project("klt" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV library: " ${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (OpenCV_FOUND)

add_executable(klt examples/klt.cpp)
target_link_libraries(klt ${OpenCV_LIBS})