// Copyright (c) 2023 Open Navigation LLC
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

#ifndef BACKPORTED_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define BACKPORTED_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "backported_bt_navigator/behavior_tree_navigator.hpp"
#include "pluginlib/class_loader.hpp"

namespace backported_bt_navigator
{
/**
 * @class backported_bt_navigator::BtNavigator
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class BtNavigator : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for backported_bt_navigator::BtNavigator class
   * @param options Additional options to control creation of the node.
   */
  explicit BtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for backported_bt_navigator::BtNavigator class
   */
  ~BtNavigator();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action servers for navigator plugins; subscription to
   * "goal_sub"; and builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // To handle all the BT related execution
  pluginlib::ClassLoader<backported_bt_navigator::NavigatorBase> class_loader_;
  std::vector<pluginlib::UniquePtr<backported_bt_navigator::NavigatorBase>> navigators_;
  backported_bt_navigator::NavigatorMuxer plugin_muxer_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  // Metrics for feedback
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;

  // Spinning transform that can be used by the node
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace backported_bt_navigator

#endif  // BACKPORTED_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
