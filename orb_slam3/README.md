# ORB-SLAM 3 (Docker)

## How to build 

```
docker build . -t slam:orbslam3
```

## How to run 

```
docker run -it --privileged --net=host --ipc=host \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e "XAUTHORITY=$XAUTH" \
    -v ~/kitti:/data \
    slam:orbslam3
```
