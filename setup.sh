#!/bin/bash

git clone https://github.com/open-mmlab/OpenPCDet
# For CUDA 12.1
torch==2.2.2+cu121 torchvision==0.17.2+cu121 torchaudio==2.2.2 --index-url https://download.pytorch.org/whl/cu121 \
python3 -m pip install tensorflow==2.19.0
python3 -m pip install spconv-cu120==2.3.6
cp ./extra/setup.py ./OpenPCDet/
cd OpenPCDet
python3 -m pip install -r requirements.txt
python3 setup.py develop
python3 -m pip install kornia==0.6.12 open3d==0.19.0
python3 -m pip install pyquaternion==0.9.9
cd ..
cd src/
git clone https://github.com/Box-Robotics/ros2_numpy -b humble
python3 -m pip install catkin_pkg
sudo apt install ros-humble-ament-cmake-nose -y
python3 -m pip install nose==1.3.7
python3 -m pip install av2==0.3.4
python3 -m pip install transform3d==0.0.4
python3 -m pip install transforms3d==0.4.2
python3 -m pip install numpy==1.26.4
sudo apt-get update && sudo apt-get install -y \
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
    libpcl-dev
cd ..
colcon build --symlink-install