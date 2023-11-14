cmake_minimum_required(VERSION 3.10)
project("BAL ceres")
set(CMAKE_CXX_STANDARD 17)

find_package(Ceres REQUIRED)

add_executable(bal_ceres examples/bal_ceres.cpp src/bal_problem.cpp)
target_include_directories(bal_ceres PUBLIC ${CERES_INCLUDE_DIRS} include)
target_link_libraries(bal_ceres ${CERES_LIBRARIES})
