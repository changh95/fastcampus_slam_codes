# SHINE-Mapping

> Original repo link: https://github.com/PRBonn/SHINE_mapping

----

## How to build

```
# Before building docker image
vim Dockerfile (and edit accordingly to select batch map building and incremental map building)

docker build . -t slam:shine_mapping
xhost +local:docker
docker run --rm -e "DISPLAY=$DISPLAY" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /media/hyunggi/Elements/dataset:/kitti -v `pwd`/configs:/configs -v `pwd`/results:/repository/results -it --runtime=nvidia --privileged slam:shine_mapping

# Visualize the results
python3 ./ply_viewer.py ./results/results/.../map/pc_map_down.ply
python3 ./ply_viewer.py ./results/results/.../map/mesh_iter_xxxx.ply
```

## Output

![](./shine-mapping.gif)
