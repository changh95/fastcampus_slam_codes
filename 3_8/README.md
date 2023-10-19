# Optical flow tracking

- uses KITTI dataset
- Detects local feature from monocular images
- Tracks features using KLT tracker

---

# How to build & run

Requirement: OpenCV4

## Local build

```
mkdir build && cd build
cmake ..
make -j
./klt ~/kitti/dataset/sequences/00/image_0 100
```

## Docker build 

Requires base build

```
docker build . -t slam:3_8
docker run -it --env DISPLAY=$DISPLAY -v ~/kitti:/data -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:3_8

# Inside docker container
cd fastcampus_slam_codes/3_8
./build/klt /data/dataset/sequences/00/image_0 100
```

---

# Output

![](./output.gif)
