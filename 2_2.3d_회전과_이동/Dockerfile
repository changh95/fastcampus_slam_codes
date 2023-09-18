FROM ubuntu:jammy

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata && \
  apt-get install -y \
  sudo \
  unzip \
  cmake \
  wget \
  build-essential \
  ninja-build \
  libglew-dev \
  libglvnd-dev \
  libgl1-mesa-dev \
  libegl1-mesa-dev \
  git \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libeigen3-dev

# Pangolin

RUN cd / && \
    wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip &&\
    unzip v0.6.zip && \
    mkdir build && \
    cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Release -GNinja ../Pangolin-0.6 &&\
    ninja -j4 && \
    ninja install

RUN git clone https://github.com/changh95/3d_pose_viewer.git && \
    cd 3d_pose_viewer &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

