# Mono-VO

> Original repository: https://github.com/avisingh599/mono-vo

- Uses KITTI dataset
- Detects FAST keypoints and tracks them using KLT tracker
- Estimates camera motion using Essential Matrix
- Scale recovery via Ground truth motion data

---

## How to build 

Requirements: OpenCV4

```bash
mkdir build
cd build
cmake ..
make
```

```docker
docker build -t slam:5:5
docker run -it --env DISPLAY=$DISPLAY -v /kitti/:/data -v /tmp/.X11-unix/:/tmp/.X11-unix:ro slam:5_5
```

## How to run

```bash
./vo path_to_image_folder path_to_gt_motion_txt_file
```

---

## Output

![](output.gif)