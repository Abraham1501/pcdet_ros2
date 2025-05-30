#!/bin/bash

# Variables for docker run
IMAGE_NAME=sdv_pcdet-ros2
CONTAINER_NAME=sdv_pcdet-ros2

DOCKER_COMMAND="docker run"

xhost +

$DOCKER_COMMAND -it -d\
    --network=host\
    --privileged \
    -v /dev:/dev \
    -v "$PWD/src:/ws/src" \
    -v "$PWD/OpenPCDet:/ws/OpenPCDet \
    --name=$CONTAINER_NAME\
    $IMAGE_NAME\
    bash