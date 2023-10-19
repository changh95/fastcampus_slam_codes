FROM slam:latest

RUN cd fastcampus_slam_codes/3_8 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

