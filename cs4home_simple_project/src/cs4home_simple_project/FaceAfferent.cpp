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

#include "cs4home_simple_project/FaceAfferent.hpp"

#include "rclcpp/rclcpp.hpp"

namespace cs4home_simple_project
{

FaceAfferent::FaceAfferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: Afferent(parent)
{
  RCLCPP_DEBUG(parent_->get_logger(), "Afferent created: [FaceAfferent]");

  // Declare the parameter that stores input topic names
  parent_->declare_parameter("simple_image_input.topics", input_topic_names_);
}

bool FaceAfferent::configure()
{
  // Get the input topic names
  parent_->get_parameter("simple_image_input.topics", input_topic_names_);

  // Create the subscription to the input topics
  for (const auto & topic_name : input_topic_names_) {
    if(this->create_subscriber(topic_name, "hri_msgs/msg/IdsList")) {
      RCLCPP_INFO(parent_->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
    } else {
      RCLCPP_ERROR(parent_->get_logger(), "Error subscribing to topic: %s", topic_name.c_str());
      return false;
    }
  }

  return true;
}

CS_REGISTER_COMPONENT(FaceAfferent)

}  // namespace cs4home_simple_project
