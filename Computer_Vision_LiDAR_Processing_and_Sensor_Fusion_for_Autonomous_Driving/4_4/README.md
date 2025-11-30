# Basic point cloud processing

- Load a lidar point cloud datum from KITTI dataset
- Use PCL functions to perform 1. Point cloud downsampling, 2. KD-tree search, 3. Passthrough filter, 4. Statistical outlier removal, 5. Plane detection

---

# How to build & run

Requirement: PCL

## Local build

```
mkdir build && cd build
cmake ..
make -j
cd ..
./build/downsampling
./build/kdtree
./build/passthrough
./build/sor
./build/plane_det
```

## Docker build 

Requires base build

```
docker build . -t slam:4_4
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:4_4

# Inside docker container
cd fastcampus_slam_codes/4_4
./build/downsampling
./build/kdtree
./build/passthrough
./build/sor
./build/plane_det
```

---

# Output

## Passthrough filter (Remove LiDAR points from vehicle)

![](./passthrough.png)

## KD-Tree (Find nearest neighbor LiDAR points)

![](./kdtree.png)

## Downsampling

![](./downsample.png)

## Floor plane detection

![](./plane_det.png)

## Statistical Outlier Removal (SOR)

![](./sor.png)
