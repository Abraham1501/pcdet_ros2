##############################################
# Created from template ros2.dockerfile.jinja
##############################################
# From athackst/dockerfiles
# https://github.com/athackst/dockerfiles/blob/main/ros2/humble-cuda.Dockerfile
# modified by Abraham1501
###########################################
# Base image
###########################################
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y --no-install-recommends tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common apt-transport-https ca-certificates && \
    add-apt-repository -y universe && \
    # Add ROS2 repository
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y ros-humble-desktop-full && \
    rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display
ENV QT_X11_NO_MITSHM=1

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=

###########################################
#  Develop image
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-humble-ament-* \
  nano \
  vim \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y --no-install-recommends git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#  Install ROS2 dependencies
###########################################
RUN apt-get update && apt-get install -y \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-vision-msgs \
    ros-humble-visualization-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf-transformations \
    ros-humble-ament-cmake-nose \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

###########################################
#  OpenPCDet Installation
###########################################

# Set GPU architecture
ENV TORCH_CUDA_ARCH_LIST="8.6"

# Set up workspace
WORKDIR /ws

# Clone repository
RUN git clone https://github.com/open-mmlab/OpenPCDet.git

# Install Python packages 
RUN python3 -m pip install \
    # For CUDA 12.1
    torch==2.2.2+cu121 torchvision==0.17.2+cu121 torchaudio==2.2.2 --index-url https://download.pytorch.org/whl/cu121 \
    && pip install spconv-cu120==2.3.6 \
    && cd OpenPCDet \
    && python3 -m pip install -r requirements.txt \
    && python3 setup.py develop \
    && python3 -m pip install \
    kornia==0.6.12 open3d==0.19.0 pyquaternion==0.9.9 \
    av2==0.3.4 \
    && rm -rf /var/lib/apt/lists/*

###############################################
# Installation and setup workspace to pcdet_ros2
###############################################

WORKDIR /ws
COPY . /ws
WORKDIR /ws/src

# clone ros_numpy
RUN git clone https://github.com/Box-Robotics/ros2_numpy -b humble

# Install Python packages
RUN python3 -m pip install catkin_pkg \
    && python3 -m pip install nose==1.3.7 transform3d==0.0.4 transforms3d==0.4.2 numpy==1.26.4 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 ament_cmake_nose dependency
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake-nose \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-vision-msgs \
    ros-humble-visualization-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf-transformations \
    ros-humble-ament-cmake-nose \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

    # Clean
RUN apt-get autoremove -y \
    && apt-get clean -y 