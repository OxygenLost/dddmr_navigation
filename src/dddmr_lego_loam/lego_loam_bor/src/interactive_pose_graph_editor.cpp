// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "interactive_pose_graph_editor.h"
#include <future>

using namespace gtsam;


InteractivePoseGraphEditor::InteractivePoseGraphEditor(std::string name, std::shared_ptr<MapOptimization> mo) : 
  Node(name), mo_(mo), name_(name)
{
  
  // operation initialization
  current_operation_nodes_.first = -1;
  current_operation_nodes_.second = -1;
  
  clock_ = this->get_clock();

  node_1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/node_1_pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  node_2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/node_2_pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  node_selection_sub_ = this->create_subscription<std_msgs::msg::String>(
      "node_selection_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&InteractivePoseGraphEditor::nodeSelectionCB, this, std::placeholders::_1));

  operation_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&InteractivePoseGraphEditor::operationCommandCB, this, std::placeholders::_1));
}

InteractivePoseGraphEditor::~InteractivePoseGraphEditor()
{
  
}

void InteractivePoseGraphEditor::convert2Global(){
  
  //@----- update keyframe poses ---
  for(auto it=cornerCloudKeyFrames_global_.begin();it!=cornerCloudKeyFrames_global_.end();it++){
    (*it).reset();
  }
  
  for(auto it=patchedGroundCloudKeyFrames_global_.begin();it!=patchedGroundCloudKeyFrames_global_.end();it++){
    (*it).reset();
  }

  cornerCloudKeyFrames_global_.clear();
  patchedGroundCloudKeyFrames_global_.clear();
  for(unsigned int it=0; it<mo_->cloudKeyPoses6D->points.size(); it++){
    pcl::PointCloud<PointType>::Ptr a_feature_pcd(new pcl::PointCloud<PointType>());
    a_feature_pcd = mo_->transformPointCloud(mo_->cornerCloudKeyFrames[it],
                           &mo_->cloudKeyPoses6D->points[it]);
    
    cornerCloudKeyFrames_global_.push_back(a_feature_pcd);

    pcl::PointCloud<PointType>::Ptr a_patched_ground_pcd(new pcl::PointCloud<PointType>());
    a_patched_ground_pcd = mo_->transformPointCloud(mo_->patchedGroundKeyFrames[it],
                           &mo_->cloudKeyPoses6D->points[it]);
    
    patchedGroundCloudKeyFrames_global_.push_back(a_patched_ground_pcd);
  }
  
}

void InteractivePoseGraphEditor::nodeSelectionCB(const std_msgs::msg::String::SharedPtr msg){
  
  if(mo_->cornerCloudKeyFrames.size()<1){
    return;
  }

  //@ convert camera_init pc to global frame
  convert2Global();

  //@ pg_0_node/88;pg_0_node/89;
  std::stringstream node_full_name(msg->data);
  std::string one_node_full_name;
  bool is_anchor = false;
  int selected_node = 0;
  while(std::getline(node_full_name, one_node_full_name, ';'))
  {

    if(one_node_full_name=="anchor"){
      is_anchor = true;
      continue;
    }

    if (one_node_full_name.find("pg_0") == std::string::npos) {
      continue;
    }

    std::stringstream one_node_full_name_stream(one_node_full_name);
    std::string node_number;
    std::vector<std::string> node_number_list;
    while(std::getline(one_node_full_name_stream, node_number, '/'))
    {
      node_number_list.push_back(node_number);
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", node_number_list[1].c_str());

    selected_node = stoi(node_number_list[1]);
    break;

  }
  
  if(is_anchor){
    current_operation_nodes_.first = selected_node;
  }
  else{
    current_operation_nodes_.second = selected_node;
  }

  if(current_operation_nodes_.first>=0){
    node_1_pointcloud_.reset(new pcl::PointCloud<PointType>());
    for(int i=current_operation_nodes_.first-3; i<current_operation_nodes_.first+4; i++){
      if(i<0 || i > cornerCloudKeyFrames_global_.size()-1)
        continue;
      *node_1_pointcloud_ += (*cornerCloudKeyFrames_global_[i]);
      //*node_1_pointcloud_ += (*patchedGroundCloudKeyFrames_global_[i]);
    }
    sensor_msgs::msg::PointCloud2 cloud1MsgTemp;
    pcl::toROSMsg(*node_1_pointcloud_, cloud1MsgTemp);
    cloud1MsgTemp.header.stamp = clock_->now();
    cloud1MsgTemp.header.frame_id = "camera_init";
    node_1_pub_->publish(cloud1MsgTemp);
  }

  if(current_operation_nodes_.second>=0){
    node_2_pointcloud_.reset(new pcl::PointCloud<PointType>());
    for(int i=current_operation_nodes_.second-1; i<current_operation_nodes_.second+1; i++){
      if(i<0 || i > cornerCloudKeyFrames_global_.size()-1)
        continue;
      *node_2_pointcloud_ += (*cornerCloudKeyFrames_global_[i]);
      //*node_2_pointcloud_ += (*patchedGroundCloudKeyFrames_global_[i]);
    }
    //node_2_pointcloud_ = cornerCloudKeyFrames_global_[current_operation_nodes_.second];
    sensor_msgs::msg::PointCloud2 cloud2MsgTemp;
    pcl::toROSMsg(*node_2_pointcloud_, cloud2MsgTemp);
    cloud2MsgTemp.header.stamp = clock_->now();
    cloud2MsgTemp.header.frame_id = "camera_init";
    node_2_pub_->publish(cloud2MsgTemp);
    
    //@ calculate node1 to node 2 tf
    PointTypePose node1 = mo_->cloudKeyPoses6D->points[current_operation_nodes_.first];
    tf2::Transform tf2_node1_pose;
    tf2::Quaternion q1;
    q1.setRPY( node1.roll, node1.pitch, node1.yaw);
    tf2_node1_pose.setRotation(q1);
    tf2_node1_pose.setOrigin(tf2::Vector3(node1.x, node1.y, node1.z));
    RCLCPP_DEBUG(this->get_logger(), "Node 1 pose: %.2f, %.2f, %.2f", tf2_node1_pose.getOrigin().x(), tf2_node1_pose.getOrigin().y(), tf2_node1_pose.getOrigin().z());    

    PointTypePose node2 = mo_->cloudKeyPoses6D->points[current_operation_nodes_.second];
    tf2::Transform tf2_node2_pose;
    tf2::Quaternion q2;
    q2.setRPY( node2.roll, node2.pitch, node2.yaw);
    tf2_node2_pose.setRotation(q2);
    tf2_node2_pose.setOrigin(tf2::Vector3(node2.x, node2.y, node2.z));
    RCLCPP_DEBUG(this->get_logger(), "Node 1 pose: %.2f, %.2f, %.2f", tf2_node2_pose.getOrigin().x(), tf2_node2_pose.getOrigin().y(), tf2_node2_pose.getOrigin().z());    
    
    tf2::Transform tf2_node12node2;
    tf2_node12node2.mult(tf2_node1_pose.inverse(), tf2_node2_pose);
    
    RCLCPP_INFO(this->get_logger(), "Node 1 to Node 2 relative pose: %.2f, %.2f, %.2f", tf2_node12node2.getOrigin().x(), tf2_node12node2.getOrigin().y(), tf2_node12node2.getOrigin().z());  
    
    // trans_node1_2_node2_.transform.translation.x = tf2_node12node2_inverse.getOrigin().x();
    // trans_node1_2_node2_.transform.translation.y = tf2_node12node2_inverse.getOrigin().y();
    // trans_node1_2_node2_.transform.translation.z = tf2_node12node2_inverse.getOrigin().z();

    // trans_node1_2_node2_.transform.rotation.x = tf2_node12node2_inverse.getRotation().x();
    // trans_node1_2_node2_.transform.rotation.y = tf2_node12node2_inverse.getRotation().y();
    // trans_node1_2_node2_.transform.rotation.z = tf2_node12node2_inverse.getRotation().z();
    // trans_node1_2_node2_.transform.rotation.w = tf2_node12node2_inverse.getRotation().w();

    trans_node1_2_node2_.transform.translation.x = 0.0;
    trans_node1_2_node2_.transform.translation.y = 0.0;
    trans_node1_2_node2_.transform.translation.z = 0.0;

    trans_node1_2_node2_.transform.rotation.x = 0.0;
    trans_node1_2_node2_.transform.rotation.y = 0.0;
    trans_node1_2_node2_.transform.rotation.z = 0.0;
    trans_node1_2_node2_.transform.rotation.w = 1.0;

    node1_2_node2_af3d_ = tf2::transformToEigen(trans_node1_2_node2_);

  }

  RCLCPP_INFO(this->get_logger(), "Current operation pair: %d, %d", current_operation_nodes_.first, current_operation_nodes_.second);

}


void InteractivePoseGraphEditor::operationCommandCB(const std_msgs::msg::String::SharedPtr msg){
  
  bool publish_icped_pc = false;

  //@-----ICP-----
  if(msg->data=="icp"){
    if(node_1_pointcloud_->points.empty() || node_2_pointcloud_->points.empty()){
      RCLCPP_INFO(this->get_logger(), "Nodes are not selected yet.");
      return;
    }
    if(current_operation_nodes_.first<0 || current_operation_nodes_.second<0){
      RCLCPP_INFO(this->get_logger(), "Nodes are not selected yet.");
      return;      
    }
    //TODO: voxel and then icp, can make result better, because icp will focus clustered area
    // ICP Settings
    /*
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    // Align clouds
    icp.setInputSource(node_2_pointcloud_);
    icp.setInputTarget(node_1_pointcloud_);
    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    //icp.hasConverged() == false
    //icp.getFitnessScore() > _history_keyframe_fitness_score)
    icp_score_ = icp.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP score: %.2f", icp_score_);
    */
    
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);

    RCLCPP_INFO(this->get_logger(), "---------------------Node 1 to Node 2 relative pose: %.2f, %.2f, %.2f", 
              tf2_ros_node1_2_node2.transform.translation.x, tf2_ros_node1_2_node2.transform.translation.y, tf2_ros_node1_2_node2.transform.translation.z);  

    pcl::PointCloud<PointType>::Ptr cloud_source_opti_transformed_ptr;
    cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f T_predict, T_final;
    T_predict.setIdentity();

    T_predict << 1.0, 0.0, 0.0, tf2_ros_node1_2_node2.transform.translation.x,
                  0.0, 1.0, 0.0, tf2_ros_node1_2_node2.transform.translation.y,
                  0.0, 0.0, 1.0, tf2_ros_node1_2_node2.transform.translation.z,
                  0.0, 0.0, 0.0, 1.0;


    OptimizedICPGN icp_opti;
    icp_opti.SetTargetCloud(node_1_pointcloud_);
    icp_opti.SetTransformationEpsilon(1e-4);
    icp_opti.SetMaxIterations(100);
    icp_opti.SetMaxCorrespondDistance(10.0);
    icp_opti.Match(node_2_pointcloud_, T_predict, cloud_source_opti_transformed_ptr, T_final);
    icp_score_ = icp_opti.GetFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP score: %.2f", icp_score_);

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f relative_rt;
    relative_rt = T_final;
    pcl::getTranslationAndEulerAngles(relative_rt, x, y, z, roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "RT ---> XYZ: %.2f, %.2f, %.2f, RPY: %.2f, %.2f, %.2f", x, y, z, roll, pitch, yaw);

    node1_2_node2_af3_ = T_final;//icp.getFinalTransformation();
    node1_2_node2_af3d_ = node1_2_node2_af3_.cast<double>();
    node_2_pointcloud_icp_.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*node_2_pointcloud_, *node_2_pointcloud_icp_, node1_2_node2_af3d_);

    sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
    pcl::toROSMsg(*node_2_pointcloud_icp_, icpedcloud2MsgTemp);
    icpedcloud2MsgTemp.header.stamp = clock_->now();
    icpedcloud2MsgTemp.header.frame_id = "camera_init";
    node_2_pub_->publish(icpedcloud2MsgTemp);
    return;
  }

  
  //@-----Accept the icp result-----
  if(msg->data=="accept"){
    
    RCLCPP_INFO(this->get_logger(), "Accept edge from: %d, %d", current_operation_nodes_.second, current_operation_nodes_.first);

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionCameraFrame;
    correctionCameraFrame = node1_2_node2_af3_;  // get transformation in camera frame
                                                  // (because points are in camera frame)
    pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);

    Eigen::Affine3f correctionLidarFrame =
        pcl::getTransformation(z, x, y, yaw, roll, pitch);
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(mo_->cloudKeyPoses6D->points[current_operation_nodes_.second]);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect =
        correctionLidarFrame *
        tWrong;  // pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom =
        Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo =
        pclPointTogtsamPose3(mo_->cloudKeyPoses6D->points[current_operation_nodes_.first]);  
    /*
    PointTypePose node2 = mo_->cloudKeyPoses6D->points[current_operation_nodes_.second];
    //Pose transform
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2::Stamped<tf2::Transform> tf2_node1_2_node2;
    tf2::fromMsg(tf2_ros_node1_2_node2, tf2_node1_2_node2);
    RCLCPP_INFO(this->get_logger(), "ICP transform: %.2f, %.2f, %.2f", tf2_node1_2_node2.getOrigin().x(), tf2_node1_2_node2.getOrigin().y(), tf2_node1_2_node2.getOrigin().z());
    
    tf2::Stamped<tf2::Transform> tf2_node2_pose;
    tf2::Quaternion q;
    q.setRPY( node2.roll, node2.pitch, node2.yaw);
    tf2_node2_pose.setRotation(q);
    tf2_node2_pose.setOrigin(tf2::Vector3(node2.x, node2.y, node2.z));
    RCLCPP_INFO(this->get_logger(), "original node 2 pose: %.2f, %.2f, %.2f", tf2_node2_pose.getOrigin().x(), tf2_node2_pose.getOrigin().y(), tf2_node2_pose.getOrigin().z());

    tf2::Transform tf2_icped_node2;
    tf2_icped_node2.mult(tf2_node1_2_node2, tf2_node2_pose); //@ here is very important, it is icp_trans*node_2 pose
    RCLCPP_INFO(this->get_logger(), "icped node 2 pose: %.2f, %.2f, %.2f", tf2_icped_node2.getOrigin().x(), tf2_icped_node2.getOrigin().y(), tf2_icped_node2.getOrigin().z());
    
    std::pair<int, int> edge;
    edge.first = current_operation_nodes_.first;
    edge.second = current_operation_nodes_.second;

    PointTypePose node1 = mo_->cloudKeyPoses6D->points[current_operation_nodes_.first];
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(node1.roll, node1.pitch, node1.yaw), Point3(node1.x, node1.y, node1.z));

    tf2::Matrix3x3 m(tf2_icped_node2.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(roll, pitch, yaw), 
                                Point3(tf2_icped_node2.getOrigin().x(), tf2_icped_node2.getOrigin().y(), tf2_icped_node2.getOrigin().z()));
    */
    // manual closure, so make it very strong
    for(int i=0;i<10;i++){
      mo_->addEdgeFromPose(current_operation_nodes_.second, current_operation_nodes_.first, poseFrom, poseTo, icp_score_);
    }
    mo_->correctPoses();
    mo_->publishKeyPosesAndFrames();

    return;
  }
  
  double step = 0.5;
  if(msg->data == "px+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2_ros_node1_2_node2.transform.translation.x += step;
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "px-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2_ros_node1_2_node2.transform.translation.x -= step;
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "py+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2_ros_node1_2_node2.transform.translation.y += step;
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "py-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2_ros_node1_2_node2.transform.translation.y -= step;
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "pz+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2_ros_node1_2_node2.transform.translation.z += step;
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "pz-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2_ros_node1_2_node2.transform.translation.z -= step;
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "roll+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    roll += 0.1;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "roll-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    roll -= 0.1;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "pitch+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    pitch += 0.1;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "pitch-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    pitch -= 0.1;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "yaw+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    yaw += 0.1;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "yaw-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    yaw -= 0.1;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    node1_2_node2_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }


  if(publish_icped_pc){
    //@ The if statement is to prevent no icp has been done
    node_2_pointcloud_icp_.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*node_2_pointcloud_, *node_2_pointcloud_icp_, node1_2_node2_af3d_);

    sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
    pcl::toROSMsg(*node_2_pointcloud_icp_, icpedcloud2MsgTemp);
    icpedcloud2MsgTemp.header.stamp = clock_->now();
    icpedcloud2MsgTemp.header.frame_id = "camera_init";
    node_2_pub_->publish(icpedcloud2MsgTemp);
  }


}
