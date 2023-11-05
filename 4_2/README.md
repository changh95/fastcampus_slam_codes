# Introduction to point cloud processing

- Load a lidar point cloud datum from KITTI dataset
- Use basic PCL data structures
- Visualize the point cloud

---

# How to build & run

Requirement: PCL

## Local build

```
mkdir build && cd build
cmake ..
make -j
./visualization
```

## Docker build 

Requires base build

```
docker build . -t slam:4_2
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:4_1

# Inside docker container
cd fastcampus_slam_codes/4_2
./build/visualization
```

---

# Output

![](./output.gif)
