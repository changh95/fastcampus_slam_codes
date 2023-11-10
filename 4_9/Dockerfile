FROM slam:latest

RUN apt-get clean && apt-get update -y && apt-get install -y qtbase5-dev qt5-qmake libqglviewer-dev-qt5

RUN cd / && \
    git clone https://github.com/OctoMap/octomap.git &&\
    cd octomap && \
    cd octomap && \
    mkdir build && cd build &&\
    cmake .. && make -j4 && make install &&\
    cd ../../octovis &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 && make install

WORKDIR /
RUN cd fastcampus_slam_codes && git pull &&\
    cd 4_9 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

