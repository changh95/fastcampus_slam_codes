FROM slam:latest

RUN cd fastcampus_slam_codes/5_17 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

