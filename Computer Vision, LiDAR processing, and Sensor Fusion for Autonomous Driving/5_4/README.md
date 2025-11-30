# Inverse perspective mapping (IPM)

- Uses KITTI dataset
- Manually select 4 points to create a bird's eye view

---

# How to build & run

Requirement: OpenCV4

## Local build

```
mkdir build && cd build
cmake ..
make -j
./bev
```

## Docker build

Requires base build

```
docker build . -t slam:5_4
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:5_4

# Inside docker container
cd fastcampus_slam_codes/5_4
./build/bev
```

---

# Output

![](./000000.png)

![](./output.png)
