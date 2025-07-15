#!/bin/bash

#-----select image
echo -n "Select image type (x64/l4t): " 
read image_type

if [[ $image_type == "x64" ]]; then 
    echo -n "Do you want to build image using cuda? (Y/N): " 
    read is_cuda
    if [ "$is_cuda" != "${is_cuda#[Yy]}" ] ;then 
        echo "----> Creating x64 image with cuda, the x64 image will be created first"
        docker build --network host -t dddmr:x64 -f Dockerfile_x64 . --no-cache
        echo "----> Starting second layer with CUDA"
        docker build --network host -t dddmr:pytorch2.5.1-cuda12.6-cudnn9-tensorrt10.7 -f Dockerfile_x64_cuda . --no-cache
    else
        echo "----> Creating x64 image without cuda"
        docker build --network host -t dddmr:x64 -f Dockerfile_x64 . --no-cache
    fi
else
    echo "----> Creating l4t image"
    docker build --network host -t dddmr:l4t_r36 -f Dockerfile_x64_l4t_r36 .
fi

