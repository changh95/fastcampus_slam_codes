FROM slam:latest

RUN cd fastcampus_slam_codes/4_3 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

