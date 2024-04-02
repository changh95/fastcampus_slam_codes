# Iterative Closest Point (ICP)

- Load a lidar point cloud datum from KITTI dataset
1. Based on known correspondences, use ICP to find the transformation between two point clouds
2. Without known correspondences, use ICP from PCL library to find the transformation between two point clouds

> In both cases, the point clouds are rotated by 10 degrees around the z-axis, and moved 2m forward.

---

# How to build & run

Requirement: PCL

## Local build

```
mkdir build && cd build
cmake ..
make -j
cd ..
./build/known_corr_icp
./build/pcl_icp
```

## Docker build 

Requires base build

```
docker build . -t slam:4_6
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:4_6

# Inside docker container
cd fastcampus_slam_codes/4_6
./build/known_corr_icp
./build/pcl_icp
```

---

# Output

## PCL_ICP

Green - Source point cloud
Red - Target point cloud
Blue - Transformed source point cloud

Purple is overlap of various colors

![](output.png)
