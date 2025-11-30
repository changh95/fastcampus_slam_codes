FROM slam:latest

RUN cd fastcampus_slam_codes/4_2 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

