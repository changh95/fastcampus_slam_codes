FROM slam:latest

RUN cd fastcampus_slam_codes/4_4 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

