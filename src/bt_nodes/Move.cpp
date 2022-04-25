// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "bt_include/Move.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys_hospital
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
    conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  try {
    node->declare_parameter("waypoints");
    node->declare_parameter("waypoint_coords");
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    // Do nothing;
  }

  if (node->has_parameter("waypoints")) {
    std::vector<std::string> wp_names;

    node->get_parameter_or("waypoints", wp_names, {});

    for (auto & wp : wp_names) {
      try {
        node->declare_parameter("waypoint_coords." + wp);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
        // Do nothing;
      }

      std::vector<double> coords;
      if (node->get_parameter_or("waypoint_coords." + wp, coords, {})) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = coords[0];
        pose.y = coords[1];
        pose.theta = coords[2];

        waypoints_[wp] = pose;
      } else {
        std::cerr << "No coordinate configured for waypoint [" << wp << "]" << std::endl;
      }
    }
  }
}

BT::NodeStatus
Move::on_tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    rclcpp::Node::SharedPtr node;
    config().blackboard->get("node", node);

    //start_time_ = node_->now();
    geometry_msgs::msg::PoseStamped goal;

    // getInput("goal", goal);
    goal.header.frame_id = "map";
    goal.header.stamp = node_->now();
    goal.pose.position.x = -1.9891;
    goal.pose.position.y = 14.6051;
    goal.pose.position.z = 0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    // goal.header.stamp = node_->now();
    // goal.header.frame_id = "/odom";
    std::cout << "Moving to waypoint***" << goal.header.frame_id << " " << std::endl; 

    goal_.pose = goal;
  }
  return BT::NodeStatus::RUNNING;

}
/*
void Move::on_wait_for_result()
{
  std::cout << "hi" << std::endl;
}
*/
BT::NodeStatus
Move::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Suceeded");

  return BT::NodeStatus::SUCCESS;
}


}  // namespace plansys_hospital

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plansys_hospital::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<plansys_hospital::Move>(
    "Move", builder);
}
