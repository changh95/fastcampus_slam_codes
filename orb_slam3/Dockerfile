FROM ubuntu:jammy 

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
RUN apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y
RUN cd / &&\
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip && \
    unzip opencv.zip && \
    cd opencv-4.4.0  && \
    mkdir build && cd build && \
    cmake -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -GNinja .. && ninja && ninja install

# Pangolin    
RUN apt-get install -y libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev libc++-dev libglew-dev libeigen3-dev
RUN cd / &&\
    wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip &&\
    unzip v0.6.zip &&\
    cd Pangolin-0.6 && mkdir build && cd build &&\
    cmake -GNinja .. &&\
    ninja &&\
    ninja install &&\
    cd ../../
    
# ORB-SLAM3
RUN apt-get clean && apt-get update -y && apt-get install -y libboost-all-dev libssl-dev
RUN wget https://github.com/UZ-SLAMLab/ORB_SLAM3/archive/refs/tags/v1.0-release.zip &&\
    unzip v1.0-release.zip &&\
    cd ORB_SLAM3-1.0-release &&\
    sh build.sh
