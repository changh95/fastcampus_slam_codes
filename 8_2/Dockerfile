FROM slam:latest

RUN apt-get install -y libgtest-dev

RUN cd fastcampus_slam_codes &&\
    git pull

RUN cd fastcampus_slam_codes/8_12 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja