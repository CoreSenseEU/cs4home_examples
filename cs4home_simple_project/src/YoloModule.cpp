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

#include "cs4home_core/CognitiveModule.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


// main function
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("cs4home_simple_project");
  std::string config_file = pkgpath + "/config/params_simple.yaml";

  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", "__node:=yolo_module", "--params-file", config_file});

  auto node = cs4home_core::CognitiveModule::make_shared(options);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}