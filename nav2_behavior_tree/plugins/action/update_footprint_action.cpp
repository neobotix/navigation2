// Copyright (c) 2022 Pradheep Padmanabhan - Neobotix GmbH
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

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/update_footprint_action.hpp"

namespace nav2_behavior_tree
{

UpdateFootprint::UpdateFootprint(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  getInput("footprint", footprint_);
  getInput("topic_name", topic_name_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  footprint_publisher_ = node_->create_publisher<geometry_msgs::msg::Polygon>(topic_name_, 10);
}

inline BT::NodeStatus UpdateFootprint::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  footprint_publisher_->publish(footprint_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::UpdateFootprint>("UpdateFootprint");
}
