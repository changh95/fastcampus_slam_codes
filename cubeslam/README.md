# CubeSLAM

## How to build 

```
docker build . -t slam:cubeslam
(For non-GPU users) docker build . -f Dockerfile_non_cuda -t slam:cubeslam

# Extract data and move the 'seq_07' folder into the project base folder, and then open 2 terminals
docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" --runtime=nvidia --volume="`pwd`:/data" --net=host --privileged slam:cubeslam

# In the first terminal
roslaunch orb_object_slam mono.launch

# In the second terminal
rosbag play /data/seq_07/left_full_gray.bag --clock -r 0.5
```

## Download data

https://drive.google.com/open?id=1FrBdmYxrrM6XeBe_vIXCuBTfZeCMgApL

## Demo

![](./output.gif)
