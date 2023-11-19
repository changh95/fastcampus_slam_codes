cmake_minimum_required(VERSION 3.10)
project(test LANGUAGES CXX)

find_package(GTest CONFIG REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/install/lib/cmake/GTest)
if(GTest_FOUND)
  message(STATUS "Found GTest - ${GTEST_INCLUDE_DIRS}")

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/install/include)
  set(GTEST_LIBS GTest::gtest)
endif(GTest_FOUND)

include_directories(src)

# Unit test
enable_testing()
add_subdirectory(test)
