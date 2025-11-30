FROM --platform=linux/amd64 ubuntu:jammy

MAINTAINER changh95
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get update -y && apt-get install build-essential -y && \
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
    apt-get install -y vim && \
    apt-get install -y zip 

RUN apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN git clone https://github.com/fdcl-gwu/aruco-markers.git &&\
    cd aruco-markers &&\
    git submodule update --init &&\
    cd libraries/opencv &&\
    mkdir build && cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules .. &&\
    make -j &&\
    make install &&\
    cd ../../

RUN cd aruco-markers/create_markers &&\
    mkdir build && cd build &&\
    cmake .. && make -j &&\
    ./generate_board -d=2 -h=10 -l=100 -m=100 -s=10 -w=20 outfile.png

RUN cd aruco-markers/pose_estimation &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j
