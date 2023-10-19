FROM slam:latest

RUN cd fastcampus_slam_codes/3_2 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

