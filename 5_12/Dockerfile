FROM slam:latest

WORKDIR /
RUN git clone https://github.com/PoseLib/PoseLib.git &&\
    cd PoseLib &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja &&\
    ninja install

RUN git clone https://github.com/tsattler/RansacLib.git &&\
    cd RansacLib &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja &&\
    ninja install

RUN cd fastcampus_slam_codes/5_4 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

