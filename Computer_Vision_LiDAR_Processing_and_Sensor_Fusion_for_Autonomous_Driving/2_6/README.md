# Introduction to Lie Group and Lie Algebra

We use Sophus library to handle Lie Group and Lie Algebra.

## How to build

Pre-requisite: Sophus

Refer to docker/Dockerfile.

## How to run (Local)

```
mkdir build && cd build
cmake ..
make
./sophus
```

## How to run (Docker)

```
# Build and run docker image
docker build . -t slam:2_6
docker run -it slam:2_6

# Inside docker
cd fastcampus_slam_codes/2_6
./build/sophus
```
