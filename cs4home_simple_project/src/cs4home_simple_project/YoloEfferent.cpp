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


#include "cs4home_core/Efferent.hpp"
#include "cs4home_core/macros.hpp"

#include "yolov8_msgs/msg/detection_array.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

class YoloEfferent : public cs4home_core::Efferent
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(YoloEfferent)

  explicit YoloEfferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
  : Efferent(parent)
  {
    RCLCPP_DEBUG(parent_->get_logger(), "Afferent created: [YoloEfferent]");

    parent_->declare_parameter("yolo_efferent.topics", output_topic_names_);
  }

  bool configure()
  {
    parent_->get_parameter("yolo_efferent.topics", output_topic_names_);

    for (size_t i = 0; i < output_topic_names_.size(); i++) {
      if (create_publisher(output_topic_names_[i], "yolov8_msgs/msg/DetectionArray")) {
        RCLCPP_DEBUG(
          parent_->get_logger(),
          "[YoloEfferent] created publisher to [%s, yolov8_msgs/msg/DetectionArray]",
          output_topic_names_[i].c_str());
      } else {
        RCLCPP_WARN(
          parent_->get_logger(),
          "[YoloEfferent] Couldn't create publisher to [%s, yolov8_msgs/msg/DetectionArray]",
          output_topic_names_[i].c_str());
      }
    }
    return true;
  }

  void publish_image(yolov8_msgs::msg::DetectionArray::UniquePtr msg)
  {
    publish(std::move(msg));
  }

private:
  std::vector<std::string> output_topic_names_;
};

CS_REGISTER_COMPONENT(YoloEfferent)
