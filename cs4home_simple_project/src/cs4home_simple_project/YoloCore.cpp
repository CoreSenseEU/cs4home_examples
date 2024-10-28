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


#include <memory>
#include <vector>
#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"

#include "cs4home_core/Core.hpp"
#include "cs4home_core/macros.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
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

  void process_in_image(std::unique_ptr<rclcpp::SerializedMessage> msg)
  {
    auto image_msg = afferent_->get_msg<sensor_msgs::msg::Image>(std::move(msg));

    int counter = std::atoi(image_msg->header.frame_id.c_str());
    counter = counter * 2;
    image_msg->header.frame_id = std::to_string(counter);

    pub_->publish(std::move(image_msg));
  }

  void process_detections(std::unique_ptr<rclcpp::SerializedMessage> msg)
  {
    auto detection_msg = afferent_->get_msg<yolov8_msgs::msg::DetectionArray>(std::move(msg));

    int counter = std::atoi(detection_msg->header.frame_id.c_str());
    counter = counter * 2;
    detection_msg->header.frame_id = std::to_string(counter);

    efferent_->publish(std::move(detection_msg));
  }

  bool configure()
  {
    RCLCPP_DEBUG(parent_->get_logger(), "Core configured");

    pub_ = parent_->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
    sub_ = parent_->create_subscription<yolov8_msgs::msg::DetectionArray>(
      "/yolo/detections", 10, std::bind(&YoloCore::process_detections, this, _1));

    return true;
  }

  bool activate()
  {
    afferent_->set_mode(
      cs4home_core::Afferent::CALLBACK, std::bind(&YoloCore::process_in_image, this, _1));

    return true;
  }

  bool deactivate()
  {
    return true;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr sub_;
};

CS_REGISTER_COMPONENT(YoloCore)
