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


#ifndef CS4HOME_SIMPLE_PROJECT__FACE_EFFERENT_HPP_
#define CS4HOME_SIMPLE_PROJECT__FACE_EFFERENT_HPP_

#include <memory>
#include <vector>
#include <string>

#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "cs4home_core/Efferent.hpp"
#include "cs4home_core/macros.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace cs4home_simple_project
{

class FaceEfferent : public cs4home_core::Efferent
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(FaceEfferent)

  explicit FaceEfferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  bool configure() override;

  void publish_graph(knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg);

private:

  std::vector<std::string> output_topic_names_;

};

}  // namespace cs4home_simple_project

#endif  // CS4HOME_SIMPLE_PROJECT__FACE_EFFERENT_HPP_
