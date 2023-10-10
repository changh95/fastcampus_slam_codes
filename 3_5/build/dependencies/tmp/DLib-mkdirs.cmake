# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/src/DLib"
  "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/src/DLib-build"
  "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/install"
  "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/tmp"
  "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/src/DLib-stamp"
  "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/src"
  "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/src/DLib-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/src/DLib-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/hyunggi/fastcampus_slam_codes/3_5/build/dependencies/src/DLib-stamp${cfgdir}") # cfgdir has leading slash
endif()
