#!/bin/bash

cd src/
git clone https://github.com/Box-Robotics/ros2_numpy -b humble
cd ..

IMAGE_NAME=sdv_pcdet-ros2_cuda12
CONTAINER_NAME=sdv_pcdet-ros2_cuda12

DOCKER_COMMAND="docker run"

xhost +

$DOCKER_COMMAND -it -d \
    --gpus all \
    --network=host \
    --ipc=host \
    --pid=host \
    --privileged \
    -v /dev:/dev \
    -v "$PWD/src:/ws/src" \
    --name=$CONTAINER_NAME \
    $IMAGE_NAME \
    bash

docker cp ./extra/setup.py sdv_pcdet-ros2_cuda12:ws/OpenPCDet
