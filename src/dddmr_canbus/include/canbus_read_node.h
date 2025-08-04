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

#ifndef EXAMPLE_ODOM_3D_H_
#define EXAMPLE_ODOM_3D_H_

#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
#include "rclcpp/rclcpp.hpp"

#include "async_can.hpp"

using namespace std::chrono_literals;

class CANBusReader: public rclcpp::Node{

  public:

    CANBusReader():Node("can_bus_reader"){init();};
    ~CANBusReader() { disconnectPort(); }

    void timerLoop();

    bool connect(std::string can_name);

  protected:

    // communication interface
    std::shared_ptr<dddmr::AsyncCAN> can_;

    // connect to roboot from CAN or serial
    using CANFrameRxCallback = dddmr::AsyncCAN::ReceiveCallback;
    bool connectPort(std::string dev_name, CANFrameRxCallback cb) {
      can_ = std::make_shared<dddmr::AsyncCAN>(dev_name);
      can_->SetReceiveCallback(cb);
      return can_->Open();
    }

    void disconnectPort() {
      if (can_ != nullptr && can_->IsOpened()) can_->Close();
    }

    void parseCANFrame(can_frame *rx_frame);

  private:
  
    void init();

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::Clock::SharedPtr clock_;
    
    rclcpp::TimerBase::SharedPtr pub_timer_; 

    
};
#endif  // EXAMPLE_ODOM_3D_H_