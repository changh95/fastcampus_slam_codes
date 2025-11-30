FROM slam:latest

RUN pip3 install open3d

RUN cd fastcampus_slam_codes/5_17 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

