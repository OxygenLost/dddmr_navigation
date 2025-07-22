#!/bin/bash

xhost +local:docker



is_cuda=$(docker image ls dddmr | grep 12.6)
is_l4t_r36=$(docker image ls dddmr | grep l4t_r36)
if [ "$is_cuda" != "" ] ;then
    docker run -it \
        --privileged \
        --network=host \
        --gpus=all \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp:/tmp" \
        --volume="/dev:/dev" \
        --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
        --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
        --name="dddmr_humble_dev" \
        dddmr:pytorch2.5.1-cuda12.6-cudnn9-tensorrt10.7
elif [ "$is_l4t_r36" != "" ] ;then 
    docker run -it \
        --privileged \
        --network=host \
        --runtime=nvidia\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --volume="/dev:/dev" \
        --volume="/tmp:/tmp" \
        --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
        --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
        --name="dddmr_humble_dev" \
        dddmr:l4t_r36
fi