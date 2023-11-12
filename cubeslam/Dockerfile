FROM nvidia/opengl:1.2-glvnd-devel-ubuntu16.04

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ARG DEBIAN_FRONTEND=noninteractive

# Dependencies          
RUN apt-get update -y && \
    apt-get install -y sudo unzip git libeigen3-dev wget build-essential gdb curl cmake \
    libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libglew-dev 
RUN apt-get install -y lsb-release &&\
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - &&\
    apt-get update -y &&\
    apt install -y ros-kinetic-desktop-full
RUN rosdep init && rosdep update
    
# OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.15.zip && \
    unzip opencv.zip && \
    cd opencv-3.4.15  && \
    mkdir build && cd build && \
    cmake .. && make -j && make install

# Pangolin
RUN wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip &&\
    unzip v0.6.zip &&\
    cd Pangolin-0.6 && mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    cd ../../

# Build cube_slam
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/changh95/cube_slam &&\
    cd cube_slam &&\
    sh ./install_dependencies.sh &&\
    cd orb_object_slam/Vocabulary &&\
    tar -zxvf ORBvoc.txt.tar.gz
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make'
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/kinetic/setup.bash"\nexec "$@"' > /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


