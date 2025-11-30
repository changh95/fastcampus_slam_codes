# How to build

```
docker build . -t superpointglue
```

# How to use

1. Start docker container

```
docker run -it --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --privileged --runtime nvidia --gpus all --volume ~/kitti:/kitti superpointglue /bin/bash
```

2. Run the code inside the container

```
cd ~/fastcampus_slam_codes/3_3
./build/superpointglue_sequence config/config.yaml /kitti/dataset/sequences/00/image_0/
```

It takes around 20 minutes to build TensorRT engine...

![](output.gif)

---

> Original repo by yuefanhao [https://github.com/yuefanhao/SuperPoint-SuperGlue-TensorRT](https://github.com/yuefanhao/SuperPoint-SuperGlue-TensorRT)


