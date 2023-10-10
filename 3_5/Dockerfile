FROM slam:latest

RUN git clone https://github.com/dorian3d/DBoW2.git &&\
    cd DBoW2 &&\
    mkdir build && cd build &&\
    cmake -GNinja .. &&\
    ninja &&\
    ninja install &&\
    cd ../../

RUN cd fastcampus_slam_codes/3_5 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

