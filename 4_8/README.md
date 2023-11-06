# Generalized ICP (G-ICP), Normal Distributions Transform (NDT), and Incremental ICP (IICP)

- Load Velodyne lidar points from KITTI dataset.
- Perform G-ICP on the LiDAR points.
- Perform NDT on the LiDAR points.
- Perform IICP on the LiDAR points.

---

# How to build & run

Requirement: PCL

## Local build

```
mkdir build && cd build
cmake ..
make -j
./gicp /data/kitti/sequences/00/velodyne
./ndt /data/kitti/sequences/00/velodyne
./iicp /data/kitti/sequences/00/velodyne
```

## Docker build 

Requires base build

```
docker build . -t slam:4_8
docker run -it --env DISPLAY=$DISPLAY -v /kitti:/data/ -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:4_8

# Inside docker container
cd fastcampus_slam_codes/4_8
./gicp /data/sequences/00/velodyne
./ndt /data/sequences/00/velodyne
./iicp /data/sequences/00/velodyne
```

---

# Output

Red: Target point cloud
Green: Transformed point cloud (Not really visible, as the points lie behind the target point cloud)

> If you can see green point clouds, it would actually mean that the registration has not worked perfectly.

## G-ICP

![gicp](./gicp.gif)

## NDT

![ndt](./ndt.gif)
