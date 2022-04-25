// Copyright 2021 Intelligent Robotics Lab
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

#ifndef BT_BEHAVIOR__MOVE_HPP_
#define BT_BEHAVIOR__MOVE_HPP_

#include <string>
#include "geometry_msgs/msg/twist.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

// #include "bt_behavior/ctrl_support/BTActionNode.hpp"
#include "plansys2_bt_actions/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/odometry.hpp"
namespace plansys_hospital
{

class Move : public plansys2::BtActionNode<
    nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus on_tick() override;
  // void on_wait_for_result() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal")
    };
  }

private:
  // rclcpp::Time start_time_;
  int goal_reached_;
  std::map<std::string, geometry_msgs::msg::Pose2D> waypoints_;

};
}  // namespace plansys_hospital

#endif  // PLANSYS_HOSPITAL__MOVE_HPP_
