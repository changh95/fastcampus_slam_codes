# Aruco pose estimation

## How to build 

```
docker build . -t slam:5_9
docker run -it --env DISPLAY=$DISPLAY --device=/dev/video0:/dev/video0 -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:5_9

# Inside docker
cd aruco-markers
vim calibration_params.yml (Edit camera calibration parameters)
cd pose_estimation/build
./pose_estimation -d=1 -l=(size_of_marker_in_meter)
```

## Demo

![](./output.gif)
