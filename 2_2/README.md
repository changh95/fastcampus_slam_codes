# 3d_pose_viewer

A simple 3D coordinate viewer. Currently supports KITTI dataset pose format.

## Demo

![](./3d_pose_viewer.gif)

## How to build

Pre-requisite: Pangolin v0.6

Refer to docker/Dockerfile.

## How to run (Local)

```
./build/pose_viewer ./cam0_to_world.txt
```

## How to run (Docker)

```
# Enable docker port for visualization
xhost +local:docker

# Build and run docker image
docker build . -t pose_viewer:latest
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro pose_viewer:latest

# Inside docker
cd 3d_pose_viewer
./build/pose_viewer ./cam0_to_world.txt
```
