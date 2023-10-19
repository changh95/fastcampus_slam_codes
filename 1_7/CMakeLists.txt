cmake_minimum_required(VERSION 3.10)
project(INTRO_TO_CPP LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

#message(STATUS "Compile options: -Wall -Wextra -Wpedantic -Werror")
#add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Terminate build if build option is invalid (i.e. Not Debug or Release)
if(CMAKE_BUILD_TYPE)
    if(NOT CMAKE_BUILD_TYPE MATCHES "Debug" AND NOT CMAKE_BUILD_TYPE MATCHES "Release")
        message(FATAL_ERROR "Invalid build type: should be either Debug or Release")
    endif(NOT CMAKE_BUILD_TYPE MATCHES "Debug" AND NOT CMAKE_BUILD_TYPE MATCHES "Release")

    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif(CMAKE_BUILD_TYPE)

# Default build type (Release)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Build type: Release (default)")
    set(CMAKE_BUILD_TYPE "Release" CACHE
            STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release") # Set the possible values of build type for cmake-gui
endif(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "-g -O0 -fPIC")
elseif(CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_CXX_FLAGS "-g -O3 -DNDEBUG -fPIC")
endif()

add_executable(vector examples/vector.cpp)
add_executable(map examples/map.cpp)
add_executable(unordered_map examples/unordered_map.cpp)
add_executable(for_loop examples/for_loop.cpp)
add_executable(while_loop examples/while_loop.cpp)
add_executable(template_function examples/template_function.cpp)
add_executable(template_class examples/template_class.cpp)

find_package(OpenCV REQUIRED)

add_executable(opencv examples/opencv.cpp)
target_link_libraries(opencv ${OpenCV_LIBS})