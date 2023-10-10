cmake_minimum_required(VERSION 3.10)
project("orb feature matching" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV library: " ${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (OpenCV_FOUND)

include_directories(${PROJECT_SOURCE_DIR}/DBoW2/DBoW2)

add_executable(bow_matching examples/bow_matching.cpp)
target_link_libraries(bow_matching
        ${OpenCV_LIBS}
        ${PROJECT_SOURCE_DIR}/DBoW2/lib/libDBoW2.so)