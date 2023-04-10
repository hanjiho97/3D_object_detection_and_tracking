FROM pytorch/pytorch:1.8.1-cuda11.1-cudnn8-devel

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC

WORKDIR /root/

# Install basics
RUN apt update -y \
    && apt install build-essential \
    && apt install -y apt-utils git curl ca-certificates bzip2 tree htop wget software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa && apt update \
    && apt install -y libglib2.0-0 libsm6 libxext6 libxrender-dev bmon iotop g++ cmake

# Install python packages
RUN PIP_INSTALL="python -m pip --no-cache-dir install" && \
    $PIP_INSTALL tensorboardX easydict pyyaml scikit-image tqdm SharedArray six

# Install Virconv requirement
RUN pip install numpy==1.23.5 && \
    pip install spconv-cu111==2.1.24 && \
    pip install prefetch_generator

# Install opencv
RUN apt update && apt install -y libgtk2.0-dev pkg-config libcanberra-gtk-module && \
    mkdir opencv && cd opencv && \
    git clone https://github.com/opencv/opencv.git && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_GENERATE_PKGCONFIG=ON ../opencv && \
    make -j8 && \
    make install && \
    cd && \
    rm -r opencv
RUN apt update && apt install -y python3-opencv && \
    pip install opencv-python

# Install eigen3
RUN apt install libeigen3-dev -y

# Install boost
RUN apt install libboost-all-dev -y

# Install spdlog
RUN git clone https://github.com/gabime/spdlog.git && \
    cd spdlog && mkdir build && cd build && \
    cmake .. && make -j && \
    make install && \
    cd && \
    rm -r spdlog

# Git clone repo
RUN git clone https://github.com/hanjiho97/3D_object_detection_and_tracking.git && \
    cd 3D_object_detection_and_tracking && mkdir build && cd build && \
    cmake .. && make -j8
    