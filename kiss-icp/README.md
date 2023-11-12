# KISS-ICP

> Original repo: https://github.com/PRBonn/kiss-icp

---

## How to build

```
docker build . -t slam:kiss_icp
```

## How to run

```
xhost +local:docker 
docker run -it --privileged --net=host --ipc=host \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e "XAUTHORITY=$XAUTH" \
    -v /kitti:/data \
    slam:kiss_icp

# Inside docker container
kiss_icp_pipeline --dataloader kitti --sequence 00 --visualize /data/
```
