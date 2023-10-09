FROM slam:latest

MAINTAINER changh95
ARG DEBIAN_FRONTEND=noninteractive

RUN cd fastcampus_slam_codes/'1_7.cpp_'$'\352\270\260\354\264\210' &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j
