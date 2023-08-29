cmake_minimum_required(VERSION 3.10)
project(
	"CV_Project"
	VERSION 0.1.0
	LANGUAGES C CXX
	)

# Standard CMake modules
include(CTest)
include(CMakeDependentOption)
include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)
include(CMakePackageConfigHelpers)

# Custom modules and scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(opencv)
include(pcl)
include(eigen)
include(ceres)
include(gtsam)
include(pangolin)
include(utils)
#include(...)

set(CMAKE_CXX_STANDARD 17)
message(STATUS "-std=C++17")

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS "-O0")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
message(STATUS "-Wall -Wextra -O0")

# 3rdParty dependencies
#add_subdirectory(third_party EXCLUDE_FROM_ALL)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# csv reader depdency
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
set(BIN_FILE solution)
set(BIN_FOLDER_PATH bin/)
set(3RDPARTY_PATH 3rdParty/)

include_directories(
    include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    include/fast-cpp-csv-parser/
    ${Eigen3_INCLUDE_DIRS}
)

add_executable(${BIN_FILE}
src/main.cpp
)

set_target_properties(${BIN_FILE} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BIN_FOLDER_PATH})

target_link_libraries(
	${OpenCV_LIBS}
)
