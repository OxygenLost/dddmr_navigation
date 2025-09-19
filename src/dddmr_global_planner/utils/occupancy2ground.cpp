/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "rclcpp/rclcpp.hpp"

#include "pcl/common/transforms.h"
#include "pcl/PCLPointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;

// Structure to hold PGM image data
typedef struct PGMImage {
  std::string magicNumber;
  int width;
  int height;
  int maxGrayValue;
  std::vector<unsigned char> pixelData; // For 8-bit grayscale
  // For 16-bit, you might use std::vector<unsigned short>
} PGMImage_t;

class Occupancy2Ground : public rclcpp::Node
{

  public:

    Occupancy2Ground();
    PGMImage_t readPGM(const std::string& filename);
    void img2Ground();

  private:

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_wall_;

    std::string map_dir_;

    PGMImage_t pgm_t_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_;
    pcl::PointCloud<pcl::PointXYZI> pc_wall_;

};


Occupancy2Ground::Occupancy2Ground():Node("occupancy2ground"){

  pc_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapground",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pub_wall_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  this->declare_parameter("map_dir", rclcpp::ParameterValue(""));
  this->get_parameter("map_dir", map_dir_);
  RCLCPP_INFO(this->get_logger(), "map_dir: %s" , map_dir_.c_str());

  if(!std::filesystem::exists(map_dir_))
  {
    RCLCPP_INFO(this->get_logger(), "File: %s not exist, exit.", map_dir_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Reading file from: %s", map_dir_.c_str());
  pgm_t_ = readPGM(map_dir_);
  img2Ground();
}

void Occupancy2Ground::img2Ground() {
  for (size_t i = 0; i < pgm_t_.pixelData.size(); ++i) {
    if(pgm_t_.pixelData[i]>200){
      pcl::PointXYZI pt;
      pt.x = (i%pgm_t_.width)*0.05;
      pt.y = pgm_t_.height*0.05 - (int)(i/pgm_t_.width)*0.05;
      pt.z = 0.0;
      pc_ground_->push_back(pt);
    }
    else{
      for(int j=0;j<5;j++){
        pcl::PointXYZI pt;
        pt.x = (i%pgm_t_.width)*0.05;
        pt.y = pgm_t_.height*0.05 - (int)(i/pgm_t_.width)*0.05;
        pt.z = j*0.2;
        pc_wall_.push_back(pt);
      }
    }
  }

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (pc_ground_);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*pc_ground_);

  sensor_msgs::msg::PointCloud2 ros_msg_map_ground;
  pcl::toROSMsg(*pc_ground_, ros_msg_map_ground);
  ros_msg_map_ground.header.frame_id = "map";
  pub_ground_->publish(ros_msg_map_ground);

  sensor_msgs::msg::PointCloud2 ros_msg_map_wall;
  pcl::toROSMsg(pc_wall_, ros_msg_map_wall);
  ros_msg_map_wall.header.frame_id = "map";
  pub_wall_->publish(ros_msg_map_wall);
  
  RCLCPP_INFO(this->get_logger(), "All cloud published");

}

PGMImage_t Occupancy2Ground::readPGM(const std::string& filename) {

    PGMImage_t image;

    std::ifstream file(filename, std::ios::binary); // Open in binary mode for both P2 and P5

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
      return image; // Return empty image
    }

    // Read header
    file >> image.magicNumber;
    if (image.magicNumber != "P2" && image.magicNumber != "P5") {
      RCLCPP_ERROR(this->get_logger(), "Invalid PGM magic number.");
      file.close();
      return image;
    }

    // Skip comments
    std::string line;
    std::getline(file, line); // Consume the rest of the magic number line
    while (file.peek() == '#') {
      std::getline(file, line);
    }

    file >> image.width >> image.height >> image.maxGrayValue;

    // Consume the newline character after maxGrayValue
    std::getline(file, line);

    // Read pixel data
    int numPixels = image.width * image.height;
    image.pixelData.resize(numPixels);

    if (image.magicNumber == "P2") { // ASCII PGM
      for (int i = 0; i < numPixels; ++i) {
        int pixelValue;
        file >> pixelValue;
        image.pixelData[i] = static_cast<unsigned char>(pixelValue);
      }
    } else if (image.magicNumber == "P5") { // Binary PGM
      file.read(reinterpret_cast<char*>(image.pixelData.data()), numPixels * sizeof(unsigned char));
    }

    file.close();

    if (!image.pixelData.empty()) {
      RCLCPP_INFO(this->get_logger(), "Width: %d, Height: %d", image.width, image.height);
      //for (int i = 0; i < std::min(10, (int)myImage.pixelData.size()); ++i) {
      //  std::cout << "Pixel " << i << ": " << (int)myImage.pixelData[i] << std::endl;
      //}
    }

    return image;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Occupancy2Ground>());
  rclcpp::shutdown();
  return 0;
}
