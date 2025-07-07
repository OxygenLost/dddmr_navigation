# DDDMR BEGINNER GUIDE

This README covers the gudie for the beginners who want to use DDDMR Navigation Stack.

## ✨ Start DDDMR Navigation with a Real Robot
You should be able to run DDDMR Navigation like a charm if your system meet following requirements:

👉 Requirements: 
  1. Your robot can be controlled by a topic (/cmd_vel) based on geometry_msgs/msg/Twist type.
  2. You have a multi layer lidar and there is a ROS2 node publishing the point cloud based on sensor_msgs/msg/PointCloud2 type.
  3. You have a ROS2 node that publishes an odometry topic and tf based on nav_msgs/msg/Odometry type.
     
     * DDDMR Navigation should work if your quadruped robot, humanoid robot or wheel robot publish reasonable odometry topic/tf, i.e. error is less than 10%.
  5. When your system is launched, you should see following tf tree. In addition, make sure your the frame_id of your lidar topic is consistent with your tf tree:
      <p align='center'>
        <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="200" height="480"/>
      </p>
  6. If you do ros2 topic list, you should be able to see: /odom, /cmd_vel, /tf and /lidar_point_cloud
  7. You are good to go.

👉 Advanced:
  1. Implement [3D odometry](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) to get a better and robust localization and mapping results.

## 🚧 Start Mapping

### Mapping from a Bag File
To map the area, you can record two ROS2 topics for the offline mapping. Recording odom topic and point cloud topic while manually drive your robot in the area.
Odom topic for the offline mapping is not mandatory, but in some cases such as featureless environment or wide open area, it can be used to improve the mapping quality.

```
ros2 bag record /odom /lidar_point_cloud
```

Once you have the bag file, modify your directory, pointcoud topic and odom topic in the off-line launch file:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_c16_config.yaml#L4
In addition, make sure the lidar spec is correctly setup:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_c16_config.yaml#L13

And then you can run the offline mapping by:

```
ros2 launch lego_loam_bor lego_loam_bag.launch
```
### Mapping in Realtime
Similar to the offline mapping, setup the lidar spec correctly in the configutation file:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/config/loam_c16_config.yaml#L4
Change the corresponding topics at:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam.launch#L6

And then you can run the realtime mapping by:
```
ros2 launch lego_loam_bor lego_loam.launch
```
## 🚧 Start Localization

## 🚧 Start Point to Point Navigation
