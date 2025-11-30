cmake_minimum_required(VERSION 3.10)
project("ransac" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV library: " ${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (OpenCV_FOUND)

add_executable(ransac_opencv examples/ransac_opencv.cpp)
target_link_libraries(ransac_opencv ${OpenCV_LIBS})

#add_executable(ransac_ransaclib examples/ransac_ransaclib.cpp)
#`target_link_libraries(ransac_ransaclib ${OpenCV_LIBS})
