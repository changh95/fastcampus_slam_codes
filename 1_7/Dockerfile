FROM slam:latest

MAINTAINER changh95
ARG DEBIAN_FRONTEND=noninteractive

RUN cd fastcampus_slam_codes/1_7 &&\
    mkdir build && cd build &&\
    cmake -GNinja .. &&\
    ninja -j4
