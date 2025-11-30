cmake_minimum_required(VERSION 3.10)
project(test LANGUAGES CXX)

find_package(GTest CONFIG REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/install/lib/cmake/GTest)

include_directories(src)

# Unit test
enable_testing()
add_subdirectory(test)
