# Global feature matching via DBoW2

- uses KITTI dataset
- detects ORB features from 5 images
- Creates vocabulary tree using DBoW2
- Matches features from 5 images using DBoW2

---

# How to build & run

Requirement: OpenCV4, DBoW2

(DBoW3, FBoW can be used as well - but API will be slightly different)

## Local build

```
mkdir build && cd build
cmake ..
make -j
./bow_matching
```

## Docker build

Requires base build

```
docker build . -t slam:3_5
docker run -it  slam:3_5

# Inside docker container
cd fastcampus_slam_codes/3_5
./build/bow_matching
```
