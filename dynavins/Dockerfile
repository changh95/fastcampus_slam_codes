FROM ros:melodic

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update -y

RUN apt-get install -y --no-install-recommends &&\
    apt-get install -y --no-install-recommends \
    wget nano build-essential ros-melodic-cv-bridge ros-melodic-pcl-ros libomp-dev libceres-dev clang lld\
    ros-melodic-geodesy ros-melodic-nmea-msgs \
    ros-melodic-rviz ros-melodic-tf-conversions

# OpenCV 
RUN apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y unzip
RUN wget https://github.com/opencv/opencv/archive/refs/tags/3.4.1.zip &&\
    unzip 3.4.1.zip &&\
    cd opencv-3.4.1 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j &&\
    make install &&\
    cd ../../

# Ceres
RUN apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev &&\
    wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.zip &&\
    unzip 1.14.0.zip &&\
    cd ceres-solver-1.14.0 && \
    mkdir build && cd build &&\
    cmake .. &&\
    make -j &&\
    make install &&\
    cd ../../

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/url-kaist/dynaVINS
RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
RUN cp VINS-Fusion/support_files/brief_k10L6.bin dynaVINS/support_files/brief_k10L6.bin
RUN cp VINS-Fusion/support_files/brief_pattern.yml dynaVINS/support_files/brief_pattern.yml
RUN mv VINS-Fusion/camera_models ./camera_models
RUN rm -rf VINS-Fusion

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

#WORKDIR /
#RUN wget https://urserver.kaist.ac.kr/publicdata/dynaVINS/d455_urban_robotics/e_shape.bag

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
