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

#include "cs4home_simple_project/FaceCore.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"


using namespace std::placeholders;

namespace cs4home_simple_project
{

FaceCore::FaceCore(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: Core(parent)
{
  RCLCPP_DEBUG(parent_->get_logger(), "Core created: [FaceCore]");
}

void FaceCore::process_in_face(std::unique_ptr<rclcpp::SerializedMessage> msg)
{
  if (!msg) {
    RCLCPP_WARN(parent_->get_logger(), "Received IdsList message with no ids.");
    return;
  }

  auto face_msgs = afferent_->get_msg<hri_msgs::msg::IdsList>(std::move(msg));

  if (face_msgs->ids.empty()) {
    RCLCPP_WARN(parent_->get_logger(), "Received IdsList message with no ids.");
    return;
  }

  auto robot_node = knowledge_graph_msgs::msg::Node();
  auto face_node = knowledge_graph_msgs::msg::Node();
  auto edge = knowledge_graph_msgs::msg::Edge();

  robot_node.node_name = "tiago";
  robot_node.node_class = "robot";

  face_node.node_name = "face_" + face_msgs->ids[0];
  face_node.node_class = "face";

  edge.source_node = robot_node.node_name;
  edge.target_node = face_node.node_name;
  edge.edge_class = "wants_to_look";

  auto graph_msg = std::make_unique<knowledge_graph_msgs::msg::GraphUpdate>();
  graph_msg->stamp = parent_->now();
  graph_msg->node_id = "face_core";
  graph_msg->operation_type = knowledge_graph_msgs::msg::GraphUpdate::REQSYNC;
  graph_msg->element_type = knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
  graph_msg->graph.nodes.push_back(robot_node);
  graph_msg->graph.nodes.push_back(face_node);
  graph_msg->graph.edges.push_back(edge);

  efferent_->publish(std::move(graph_msg));

}

bool FaceCore::configure()
{
  RCLCPP_DEBUG(parent_->get_logger(), "Core configured");
  this->graph_ = knowledge_graph::KnowledgeGraph::get_instance(parent_->shared_from_this());
  return true;
}

bool FaceCore::activate()
{
  afferent_->set_mode(
    cs4home_core::Afferent::CALLBACK, std::bind(&FaceCore::process_in_face, this, _1));
  RCLCPP_DEBUG(parent_->get_logger(), "Core activated");
  return true;
}

bool FaceCore::deactivate()
{
  RCLCPP_DEBUG(parent_->get_logger(), "Core deactivated");
  timer_ = nullptr;
  return true;
}

}  // namespace cs4home_simple_project

CS_REGISTER_COMPONENT(cs4home_simple_project::FaceCore)
