FROM slam:latest

RUN cd fastcampus_slam_codes/5_5 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja