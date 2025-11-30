cmake_minimum_required(VERSION 3.6)
project(tensorrtbuffer)

set(CMAKE_CXX_STANDARD 14)

find_package(CUDA REQUIRED)
include_directories("${CUDA_INCLUDE_DIRS}")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(tensorrtbuffer SHARED src/logger.cpp)



