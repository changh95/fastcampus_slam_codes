# Essential / Fundamental matrix estimation

- uses KITTI dataset
- Detects local feature from monocular images
- Uses OpenCV functions to estimate E/F matrix

---

# How to build & run

Requirement: OpenCV4

## Local build

```
mkdir build && cd build
cmake ..
make -j
./epipolar
```

## Docker build 

Requires base build

```
docker build . -t slam:5_2
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:5_2

# Inside docker container
cd fastcampus_slam_codes/5_2
./build/epipolar
```

---

# Output

![](./output.gif)
