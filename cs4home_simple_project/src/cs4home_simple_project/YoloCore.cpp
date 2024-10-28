// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "cs4home_core/Core.hpp"
#include "cs4home_core/macros.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class YoloCore : public cs4home_core::Core
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(YoloCore)

  explicit  YoloCore(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
  : Core(parent)
  {
    RCLCPP_DEBUG(parent_->get_logger(), "Core created: [YoloCore]");
  }

  void process_in_image(sensor_msgs::msg::Image::UniquePtr msg)
  {
    int counter = std::atoi(msg->header.frame_id.c_str());
    counter = counter * 2;
    msg->header.frame_id = std::to_string(counter);

    efferent_->publish(std::move(msg));
  }

  void timer_callback()
  {
    auto msg = afferent_->get_msg<sensor_msgs::msg::Image>();
    if (msg != nullptr) {
      process_in_image(std::move(msg));
    }
  }

  bool configure()
  {
    RCLCPP_DEBUG(parent_->get_logger(), "Core configured");
    return true;
  }

  bool activate()
  {
    timer_ = parent_->create_wall_timer(
      50ms, std::bind(&YoloCore::timer_callback, this));
    return true;
  }

  bool deactivate()
  {
    timer_ = nullptr;
    return true;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

CS_REGISTER_COMPONENT(YoloCore)
