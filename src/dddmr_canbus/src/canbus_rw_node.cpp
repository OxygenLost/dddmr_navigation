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

#include "canbus_rw_node.h"

using std::placeholders::_1;

void CANBusReader::init(){

  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group_;
  clock_ = this->get_clock();
  
  pub_timer_ = this->create_wall_timer(100ms, std::bind(&CANBusReader::timerLoop, this), cb_group_);
  connect("can0");
}

bool CANBusReader::connect(std::string can_name){
  return connectPort(can_name,
                    std::bind(&CANBusReader::parseCANFrame, this,
                              std::placeholders::_1));
}

void CANBusReader::parseCANFrame(can_frame *rx_frame) {
  
  //@ candump id is hex, here the rx_frame_id is decimal. 
  //@ can0  262   [8]  01 24 00 1E 1E 00 00 00 ---> rx_frame->can_id = 610  (262-->610)
  //RCLCPP_INFO(this->get_logger(), "frame_id: %d, %u", rx_frame->can_id, std::make_unsigned_t<int>(rx_frame->can_id));
    
  if(std::make_unsigned_t<int>(rx_frame->can_id) == 651){
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[0]));
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[1]));
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[2]));
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[3]));
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[4]));
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[5]));
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[6]));
    RCLCPP_INFO(this->get_logger(), "%d", static_cast<int>(rx_frame->data[7]));
    RCLCPP_INFO(this->get_logger(), "----------------");
  }

  //@ first digit is wrong when frane_id is 32 digit.
  //@ https://github.com/brutella/can/issues/16
  //@ candump frame_id: 0CFDE900(217966848) -->received is 8CFDE900(2365450496)
  if(std::make_unsigned_t<int>(rx_frame->can_id) == 217962548){
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[0]));
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[1]));
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[2]));
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[3]));
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[4]));
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[5]));
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[6]));
    RCLCPP_INFO(this->get_logger(), "s %d", static_cast<int>(rx_frame->data[7]));
    RCLCPP_INFO(this->get_logger(), "----------------");
  }
  

}

void CANBusReader::timerLoop(){
  return; 
  can_frame frame;

  frame.can_id = 304;
  frame.can_dlc = 8;

  int16_t linear_cmd =
      (int16_t)(0.1 * 1000);
  int16_t angular_cmd =
      (int16_t)(0.2 * 1000);
  int16_t lateral_cmd =
      (int16_t)(0.3 * 1000);
  int16_t steering_cmd =
      (int16_t)(0.4 * 1000);

  frame.data[0] = (uint8_t)(linear_cmd >> 8);
  frame.data[1] = (uint8_t)(linear_cmd & 0x00ff);
  frame.data[2] = (uint8_t)(angular_cmd >> 8);
  frame.data[3] = (uint8_t)(angular_cmd & 0x00ff);
  frame.data[4] = (uint8_t)(lateral_cmd >> 8);
  frame.data[5] = (uint8_t)(lateral_cmd & 0x00ff);
  frame.data[6] = (uint8_t)(steering_cmd >> 8);
  frame.data[7] = (uint8_t)(steering_cmd & 0x00ff);

  // encode msg to can frame and send to bus
  if (can_ != nullptr && can_->IsOpened()) {
    
    can_->SendFrame(frame);
  }
}



int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  auto CBR = std::make_shared<CANBusReader>();
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(); 
  executor->add_node(CBR); 
  executor->spin();
  rclcpp::shutdown();  
  return 0;
}


