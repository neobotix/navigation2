// Copyright (c) Pradheep Padmanabhan, 2022 Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_FOOTPRINT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_FOOTPRINT_ACTION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/polygon.hpp"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to update the local and the global footprint
 */
class UpdateFootprint : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::UpdateFootprint constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  UpdateFootprint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Polygon>("footprint", "footprint to be defined for local and global costmap"),
      BT::InputPort<std::string>("topic_name", "topic name"),

    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr footprint_publisher_;
  geometry_msgs::msg::Polygon footprint_;
  std::string topic_name_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_FOOTPRINT_ACTION_HPP_