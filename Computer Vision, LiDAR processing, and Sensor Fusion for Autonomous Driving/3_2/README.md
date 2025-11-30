# Feature detection and matching

- uses KITTI dataset
- detects ORB features from stereo images
- matches feature via Brute-force / KNN matching

---

# How to build & run

Requirement: OpenCV4

## Local build

```
mkdir build && cd build
cmake ..
make -j
./orb_feature_matching ~/kitti/dataset/sequences/00/image_0 ~/kitti/dataset/sequences/00/image_1 100
```

## Docker build 

Requires base build

```
docker build . -t slam:3_2
docker run -it --env DISPLAY=$DISPLAY -v ~/kitti:/data -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:3_2

# Inside docker container
cd fastcampus_slam_codes/3_2
./build/orb_feature_matching /data/dataset/sequences/00/image_0 /data/dataset/sequences/00/image_1 100
```

---

# Output

![](./output.gif)
