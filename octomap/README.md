# How to build

```
docker build . -tag octomap
```

# How to run

```
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro octomap
```
