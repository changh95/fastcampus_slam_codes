# Octree & Octomap

- Load a lidar point cloud datum from KITTI dataset
1. Perform octree-based nearest neighbor search
2. Make voxel map based on octomap library

---

# How to build & run

Requirement: PCL, octomap

## Local build

```
mkdir build && cd build
cmake ..
make -j
./octree
./octomap
```

## Docker build 

Requires base build

```
docker build . -t slam:4_9
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:4_6

# Inside docker container
cd fastcampus_slam_codes/4_9
./octree
./octomap
```

---

# Output

## Octree

![](octree.png)

## Octomap

![](octomap.png)