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

#include "cs4home_simple_project/FaceEfferent.hpp"

namespace cs4home_simple_project
{

FaceEfferent::FaceEfferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: Efferent(parent)
{
  RCLCPP_DEBUG(parent_->get_logger(), "Efferent created: [FaceEfferent]");

  // Declare and load parameters for output topic names
  parent_->declare_parameter("face_efferent.output_topics", output_topic_names_);
}

bool FaceEfferent::configure()
{
  parent_->get_parameter("face_efferent.output_topics", output_topic_names_);


  // Set up publishers for each output topic
  for (const auto & topic_name : output_topic_names_) {
    if (create_publisher(topic_name, "knowledge_graph/msg/GraphUpdate")) {
      RCLCPP_INFO(
        parent_->get_logger(),
        "[FaceEfferent] Created publisher on [%s, knowledge_graph/msg/GraphUpdate]",
        topic_name.c_str());
    } else {
      RCLCPP_WARN(
        parent_->get_logger(),
        "[FaceEfferent] Couldn't create publisher on [%s, knowledge_graph/msg/GraphUpdate]",
        topic_name.c_str());
    }
  }

  return true;
}

void FaceEfferent::publish_graph(knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg)
{
  publish(std::move(msg));
}

CS_REGISTER_COMPONENT(FaceEfferent)

}  // namespace cs4home_simple_project
