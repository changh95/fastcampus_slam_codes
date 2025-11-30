FROM slam:latest

RUN cd fastcampus_slam_codes/2_6 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

