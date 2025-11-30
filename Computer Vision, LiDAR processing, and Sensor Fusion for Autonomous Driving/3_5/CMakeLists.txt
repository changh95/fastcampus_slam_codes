cmake_minimum_required(VERSION 3.10)
project("orb feature matching" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV library: " ${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (OpenCV_FOUND)

set(DBoW2_INCLUDE_DIRS "/usr/local/include")
set(DBoW2_LIBS "/usr/local/lib/libDBoW2.so")

include_directories(${DBoW2_INCLUDE_DIRS})

add_executable(bow_matching examples/bow_matching.cpp)
target_link_libraries(bow_matching
        ${OpenCV_LIBS}
        ${DBoW2_LIBS})