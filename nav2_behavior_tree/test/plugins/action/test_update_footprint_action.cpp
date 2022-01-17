// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/update_footprint_action.hpp"

using namespace std::literals::chrono_literals; // NOLINT

using std::placeholders::_1;

class UpdateFootprintSubscriber: public rclcpp::Node
{
public:
  explicit UpdateFootprintSubscriber()
  : Node("update_footprint")
  {
    sub = this->create_subscription<geometry_msgs::msg::Polygon>(
      "test/footprint", 1, std::bind(&UpdateFootprintSubscriber::topic_callback, this,_1));
  }
  void topic_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
  {
      latestMsg = msg;
  }

  geometry_msgs::msg::Polygon::SharedPtr latestMsg;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub;
};

class UpdateFootprintTestFixture : public ::testing::Test
{
public:
  geometry_msgs::msg::Polygon::SharedPtr latestMsg;

  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("update_footprint_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();


    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::UpdateFootprint>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::UpdateFootprint>(
      "UpdateFootprint", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  // void topic_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
  // {
  //     latestMsg = msg;
  // }

  std::shared_ptr<UpdateFootprintSubscriber> subscriber_;
  // rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr UpdateFootprintTestFixture::node_ = nullptr;

BT::NodeConfiguration * UpdateFootprintTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> UpdateFootprintTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> UpdateFootprintTestFixture::tree_ = nullptr;

TEST_F(UpdateFootprintTestFixture, test_tick)
{
  // sub = node_->create_subscription<geometry_msgs::msg::Polygon>(
  //     "test/footprint", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&UpdateFootprintTestFixture::topic_callback, this,_1));
  
  // create tree
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(subscriber_);
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <UpdateFootprint name="FootprintUpdate" footprint="0.1;0.4,-0.1;0.4,-0.1;-0.4,0.1;-0.4" topic_name="test/footprint"/>
        </BehaviorTree>
      </root>)";
  
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  tree_->rootNode()->executeTick(); 
  executor.spin();
  geometry_msgs::msg::Polygon msg_;
  EXPECT_EQ(msg_, *subscriber_->latestMsg);  
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // std::shared_ptr<UpdateFootprintSubscriber> subscriber_;

  // subscriber_ = std::make_shared<UpdateFootprintSubscriber>();


  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
