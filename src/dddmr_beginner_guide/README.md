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

## 🚧 Start Localization

## 🚧 Start Point to Point Navigation
