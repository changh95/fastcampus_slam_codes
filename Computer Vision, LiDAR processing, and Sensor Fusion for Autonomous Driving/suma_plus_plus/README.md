# Suma++

## How to run
```
docker build -t slam:sumapp
docker run -it -e "DISPLAY=$DISPLAY" -e "QT_X11_NO_MITSHM=1" -e "XAUTHORITY=$XAUTH" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /kitti:/data --runtime=nvidia --net=host --ipc=host --privileged slam:sumapp

# Inside container
cd semantic_suma
vim config/default.xml (Edit the content as below)
cd bin
./visualizer
```

## Visualizer

![](Visualizer.png)

1. Click 'Open Laserscan'
2. Select a velodyne scan from KITTI dataset (located at /data).
3. Wait a few minutes whilst ONNX model gets converted into TensorRT model.
4. Press 'play' at the bottom

## config/default.xml

```
...
<param name="model_path" type="string">/catkin_ws/src/semantic_suma/darknet53/</param>
...
```

## Output

![](./output.gif)
