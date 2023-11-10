FROM slam:latest

RUN apt-get update && apt-get install -y libboost-all-dev
RUN git clone https://github.com/MIT-SPARK/TEASER-plusplus.git &&\
    cd TEASER-plusplus && mkdir build && cd build &&\
    cmake .. && make &&\
    make install

RUN cd fastcampus_slam_codes/4_8 &&\
    mkdir build && cd build && \
    cmake -GNinja ../ && \
    ninja

