FROM slam:latest

RUN cd fastcampus_slam_codes/2_2.3d_회전과_이동 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja -j4

