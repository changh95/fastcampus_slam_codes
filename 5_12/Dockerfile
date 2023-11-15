FROM slam:latest

WORKDIR /
#RUN git clone https://github.com/PoseLib/PoseLib.git &&\
#    cd PoseLib &&\
#    mkdir build install && cd build && \
#    cmake -DCMAKE_INSTALL_PREFIX=../install -GNinja ../ && \
#    ninja &&\
#    ninja install

#RUN git clone https://github.com/tsattler/RansacLib.git &&\
#    cd RansacLib &&\
#    git submodule update --init --recursive &&\
#    mkdir build && cd build && \
#    cmake -DCMAKE_PREFIX_PATH=/PoseLib/install/lib/cmake/PoseLib -GNinja ../ && \
#    ninja &&\
#    ninja install

RUN cd fastcampus_slam_codes/5_4 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

