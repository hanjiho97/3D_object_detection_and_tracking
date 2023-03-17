FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC && \
    apt update && apt install -y lsb-core software-properties-common apt-utils wget curl && \
    apt install -y python2.7 python-pip && python2 -m pip install --upgrade pip

WORKDIR /root/

# install ROS-melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && apt install -y ros-melodic-desktop-full && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && source ~/.bashrc && \
    apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential && \
    rosdep init && sudo rosdep fix-permissions && rosdep update

# install carla-ros bridge

RUN source ~/.bashrc && apt update && apt install -y python-catkin-tools && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9 && \
    add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main" && \
    mkdir -p /root/catkin_ws/src && \
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git catkin_ws/src/ros-bridge && \
    cd /root/catkin_ws/ && \
    sudo rosdep fix-permissions && rosdep update && rosdep install --from-paths src --ignore-src -r && \
    catkin build && \
    python2 -m pip install carla==0.9.13 pygame && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# install python3 and pytorch

RUN apt install -y python3-pip && python3 -m pip install --upgrade pip && \
    python3 -m pip install torch==1.10.1+cu102 torchvision==0.11.2+cu102 torchaudio==0.10.1 -f https://download.pytorch.org/whl/cu102/torch_stable.html


