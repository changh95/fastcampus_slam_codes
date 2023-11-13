# RANSAC, OpenCV USAC, RansacLib

- Uses KITTI dataset
- Match left/right images using RANSAC, OpenCV USAC, [RansacLib](https://github.com/tsattler/RansacLib/tree/master)

---

# How to build & run

Requirement: OpenCV4, [RansacLib](https://github.com/tsattler/RansacLib/tree/master), [PoseLib](https://github.com/PoseLib/PoseLib)

## Local build

```
mkdir build && cd build
cmake ..
make -j
./ransac_opencv
./ransac_ransac_lib
```

## Docker build

Requires base build

```
docker build . -t slam:5_12
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:5_12

# Inside docker container
cd fastcampus_slam_codes/5_12
./ransac_opencv
./ransac_ransac_lib
```

---

## Demo (Inlier visualization)

![demo](./output.png)