# Triangulation

- Uses KITTI dataset
- Detects ORB features from stereo images
- Triangulates 3D points from stereo images

---

# How to build & run

Requirement: OpenCV4, Pangolin, Eigen3

## Local build

```
mkdir build && cd build
cmake ..
make -j
./triangulation ~/kitti/sequences/00/image_0/ ~/kitti/sequences/00/image_1/ 1000
```

## Docker build

Requires base build

```
docker build . -t slam:5_7
docker run -it --env DISPLAY=$DISPLAY -v ~/kitti:/data -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:5_7

# Inside docker container
cd fastcampus_slam_codes/5_7
./build/kitti /data/sequences/00/image_0/ /data/sequences/00/image_1/ 1000
```

---

# Output

![](./output.gif)
