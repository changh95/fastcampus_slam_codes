# CUDA 11.7, TensorRT 8.4.1
FROM nvcr.io/nvidia/tensorrt:22.07-py3 

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install libeigen3-dev wget build-essential cmake && \
    # OpenCV
    apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip && \
    unzip opencv.zip && \
    cd opencv-4.4.0  && \
    mkdir build && cd build && \
    cmake .. && make -j4 && make install

RUN apt-get install libyaml-cpp-dev -y
    
RUN cd / && \
    git clone https://github.com/changh95/fastcampus_slam_codes.git && \
    cd fastcampus_slam_codes/3_3 &&\
    mkdir build && cd build && cmake .. && make -j4
