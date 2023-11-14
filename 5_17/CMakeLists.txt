cmake_minimum_required(VERSION 3.10)
project("BAL ceres")
set(CMAKE_CXX_STANDARD 17)

find_package(Ceres REQUIRED)
if (Ceres_FOUND)
    message(STATUS "Found OpenCV library: " ${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (Ceres_FOUND)

add_executable(bal_ceres examples/bal_ceres.cpp examples/common.cpp)
target_include_directories(bal_ceres PUBLIC ${CERES_INCLUDE_DIRS} examples)
target_link_libraries(bal_ceres ${CERES_LIBRARIES})
