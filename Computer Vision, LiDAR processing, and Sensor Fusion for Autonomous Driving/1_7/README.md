# Introduction to C++ programming for SLAM

This is a repository for introductory lessons on C++ programming for SLAM.

# How to build

Dependencies: OpenCV

Local build
```
mkdir build 
cd build
cmake ..
make -j
```

Docker build
```
docker build . -t slam:1_7
```

# How to run

Local
```
./build/for_loop
./build/map
```

Docker
```
xhost +local:docker
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro slam:1_7
```

---

## CMakeLists.txt

Make sure you understand the followings:

- cmake_minimum_required
- project
- CMAKE_BUILD_TYPE
- if/elseif/endif
- add_executable
- add_library
- target_link_libraries
- find_package

---
## C++ basics

Make sure you understand the followings:

- for_loop
- while_loop
- template_class
- template_function
- vector
- map
- unordered_map
- opencv basics

---

## Docker

Make sure you understand the followings:

- Docker image / container concepts
- How to build docker image
- How to run docker container
  - with X11 forwarding
  - with volume binding

