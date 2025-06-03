#!/bin/bash

git clone https://github.com/open-mmlab/OpenPCDet
python3 -m pip install torch torchvision torchaudio
python3 -m pip install tensorflow
python3 -m pip install spconv-cu117
cd OpenPCDet
python3 -m pip install -r requirements.txt
python3 setup.py develop
python3 -m pip install kornia open3d
python3 -m pip install pyquaternion
cd ..
cd src/
git clone https://github.com/Box-Robotics/ros2_numpy -b humble
python3 -m pip install catkin_pkg
sudo apt install ros-humble-ament-cmake-nose -y
python3 -m pip install nose
python3 -m pip install transform3d
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install