FROM --platform=linux/amd64 ubuntu:noble

MAINTAINER changh95
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get install build-essential -y && \
    apt-get install cmake -y && \
    apt-get install git -y && \
    apt-get install sudo -y && \
    apt-get install wget -y && \
    apt-get install ninja-build -y && \
    apt-get install software-properties-common -y && \
    apt-get install python3 -y && \
    apt-get install python3-pip -y && \
    apt-get install -y ssh && \
    apt-get install -y gcc && \
    apt-get install -y g++ && \
    apt-get install -y gdb && \
    apt-get install -y cmake && \
    apt-get install -y rsync && \
    apt-get install -y tar && \
    apt-get install -y x11-utils && \
    apt-get install -y x11-apps && \
    apt-get install -y zip &&\
    apt-get clean

# OpenCV
RUN apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN wget https://github.com/opencv/opencv/archive/refs/tags/4.12.0.zip &&\
    unzip 4.12.0.zip &&\
    cd opencv-4.12.0 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j8 &&\
    make install &&\
    cd ../../

# Eigen
RUN wget https://gitlab.com/libeigen/eigen/-/archive/5.0.0/eigen-5.0.0.zip &&\
    unzip eigen-5.0.0.zip &&\
    cd eigen-5.0.0 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j8 &&\
    make install &&\
    cd ../../

# Ceres-solver [Use the latest branch, as latest release (2.2.0) does not support Eigen 5.0.0]
RUN apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev &&\
    git clone https://github.com/ceres-solver/ceres-solver.git --recursive &&\
    cd ceres-solver && \
    mkdir build && cd build &&\
    cmake .. &&\
    make -j8 &&\
    make install &&\
    cd ../../

# Sophus [EDIT: Remove 3.4.0 dependency to enable build with Eigen 5.0.0]
RUN wget https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.zip &&\
    unzip 1.24.6.zip &&\
    cd Sophus-1.24.6 &&\
    sed -i 's/find_package(Eigen3 3.4.0/find_package(Eigen3/g' CMakeLists.txt &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j8 &&\
    make install && \
    cd ../../

# Pangolin
RUN apt-get update && \
    apt-get install -y mesa-utils \
    libgl1 \
    libglu1-mesa-dev \
    libglew-dev \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    mesa-common-dev \
    libepoxy-dev

RUN wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.9.4.zip &&\
    unzip v0.9.4.zip &&\
    cd Pangolin-0.9.4 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j8 &&\
    make install &&\
    cd ../../
    
# PCL
RUN apt-get install -y libpcl-dev


RUN git clone https://github.com/changh95/fastcampus_slam_codes.git
