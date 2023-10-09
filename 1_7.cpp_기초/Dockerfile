FROM --platform=linux/amd64 ubuntu:jammy

MAINTAINER changh95
ARG DEBIAN_FRONTEND=noninteractive

RUN cd fastcampus_slam_codes/1_7.cpp_기초 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j
