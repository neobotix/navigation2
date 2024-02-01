// Copyright (c) 2024 Neobotix GmbH
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

#ifndef NAV2_RVIZ_PLUGINS__SELECTOR_HPP_
#define NAV2_RVIZ_PLUGINS__SELECTOR_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#include <QFrame>
#include <QGridLayout>
#include <QScrollArea>
#include <QToolButton>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "vector"
#include "memory"
#include "string"
#include "std_msgs/msg/string.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{
class Selector : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Selector(QWidget * parent = 0);
  ~Selector();

private:
  // The (non-spinning) client node used to invoke the action client
  void timerEvent(QTimerEvent * event) override;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_controller_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_planner_;
  rclcpp::TimerBase::SharedPtr rclcpp_timer_;

  bool plugins_loaded_ = false;

  QBasicTimer timer_;
  QVBoxLayout * main_layout;
  QComboBox * controller_;
  QComboBox * planner_;

  void start_ros_timer();
  void setController();
  void setPlanner();

  /*
    * @brief Load the avaialble plugins into the combo box
    * @param node The node to use for loading the plugins
    * @param server_name The name of the server to load plugins for
    * @param plugin_type The type of plugin to load
    * @param combo_box The combo box to add the loaded plugins to
  */
  void pluginLoader(
    rclcpp::Node::SharedPtr node,
    const std::string & server_name,
    const std::string & plugin_type,
    QComboBox * combo_box);

protected:
  QVBoxLayout * layout1 = new QVBoxLayout;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__SELECTOR_HPP_
