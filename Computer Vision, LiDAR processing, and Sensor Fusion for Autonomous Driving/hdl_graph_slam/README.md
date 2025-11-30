# HDL-Graph-SLAM

> Original Repo: https://github.com/koide3/hdl_graph_slam/tree/master

- Runs hdl_400.bag dataset, located at `/hdl_400.bag` inside the docker image.

## How to build

```
docker build . -t slam:hdl_graph_slam
```

## How to run

You need to open 4 terminals, and type below commands in each terminal.

```
# In every terminal, type the commands below.
xhost +local:root
docker run -it --net=host --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix slam:hdl_graph_slam
```

```
# First terminal
roscore
```

```
# Second terminal
rosparam set use_sim_time true
cd /root/catkin_ws/src/hdl_graph_slam/rviz/
rviz -d hdl_graph_slam.rviz
```

```
# Third terminal
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```

```
# Fourth terminal
rosbag play --clock /hdl_400.bag

```

## Results

![](output.gif)
