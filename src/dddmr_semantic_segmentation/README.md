# dddmr_semantic_segmentation

DDDMR semantic segmentation convert semantic segmentation result and align it with depth image to colorize the point cloud.
The TensorRT is used to faciliate the inference speed, we can achieve ~15 fps of resulted point cloud.
 
Generate TensorRT file from the onnx:
```
cd /root/dddmr_navigation/src/dddmr_semantic_segmentation/model
/usr/src/tensorrt/bin/trtexec --onnx=ddrnet_23_slim_dualresnet_citys_best_model_424x848.onnx --saveEngine=ddrnet_23_slim_dualresnet_citys_best_model_424x848.trt
```

Request:
This pipeline requires RGBD camera.

Compile
```
source /opt/ros/humble/install/setup.bash
cd /root/dddmr_navigation
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch dddmr_semantic_segmentation rs_semantic_segmentaton_trt_launch.py
```

