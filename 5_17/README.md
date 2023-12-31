# Bundle adjustment on BAL dataset


---

# How to build & run

Requirement: Ceres-solver

## Local build

```
mkdir build && cd build
cmake ..
make -j
./bal_ceres --input=../bal_data.txt
```

## Docker build

Requires base build

```
docker build . -t slam:5_17
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:5_17

# Inside docker container
cd fastcampus_slam_codes/5_17
./build/bal_ceres --input=./bal_data.txt
```

---

## Demo

![demo](./output.png)
