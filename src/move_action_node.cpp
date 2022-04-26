// Copyright 2022 RoboRos
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

#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 250ms)
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = now();
    wp.pose.position.x = 0.0;
    wp.pose.position.y = -2.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = -1.0;
    waypoints_["left_elevator"] = wp;

    wp.pose.position.x = -1.9891;
    wp.pose.position.y = 14.6051;
    waypoints_["right_elevator"] = wp;

    wp.pose.position.x = 4.12600;
    wp.pose.position.y = 18.7558;
    waypoints_["upstairs"] = wp;

    wp.pose.position.x = 6.2255;
    wp.pose.position.y = 11.8669;
    waypoints_["storage_room_1"] = wp;

    wp.pose.position.x = 8.8666;
    wp.pose.position.y = 10.2101;
    waypoints_["high_dependency_room_1"] = wp;

    wp.pose.position.x = 8.4302;
    wp.pose.position.y = 4.0321;
    waypoints_["high_dependency_room_2"] = wp;

    wp.pose.position.x = 5.8101;
    wp.pose.position.y = -9.7509;
    waypoints_["intensive_care_unit_1"] = wp;

    wp.pose.position.x = 5.9907;
    wp.pose.position.y = -22.4976;
    waypoints_["intensive_care_unit_2"] = wp;

    wp.pose.position.x = 4.7175;
    wp.pose.position.y = -26.0192;
    waypoints_["intensive_care_unit_3"] = wp;

    wp.pose.position.x = -7.3899;
    wp.pose.position.y = -24.4725;
    waypoints_["day_room"] = wp;

    wp.pose.position.x = -7.0955;
    wp.pose.position.y = -22.5098;
    waypoints_["intensive_care_unit_4"] = wp;

    wp.pose.position.x = -7.2525;
    wp.pose.position.y = -9.6043;
    waypoints_["intensive_care_unit_5"] = wp;

    wp.pose.position.x = -9.8253;
    wp.pose.position.y = 4.0447;
    waypoints_["high_dependency_room_3"] = wp;

    wp.pose.position.x = -9.8850;
    wp.pose.position.y = 10.4173;
    waypoints_["high_dependency_room_4"] = wp;

    wp.pose.position.x = -7.3516;
    wp.pose.position.y = 11.7561;
    waypoints_["storage_room_1"] = wp;

    wp.pose.position.x = 1.4548;
    wp.pose.position.y = 6.0142;
    waypoints_["reception"] = wp;

    wp.pose.position.x = 4.4300;
    wp.pose.position.y = 9.3196;
    waypoints_["waiting_area_1"] = wp;

    wp.pose.position.x = -6.6702;
    wp.pose.position.y = 9.4974;
    waypoints_["waiting_area_2"] = wp;

    wp.pose.position.x = 1.4375;
    wp.pose.position.y = -15386;
    waypoints_["consulting_room_1"] = wp;

    wp.pose.position.x = -2.9435;
    wp.pose.position.y = -1.3230;
    waypoints_["consulting_room_2"] = wp;

    wp.pose.position.x = 1.3243;
    wp.pose.position.y = -10.8608;
    waypoints_["washroom_1"] = wp;

    wp.pose.position.x = -3.2162;
    wp.pose.position.y = -10.7512;
    waypoints_["washroom_2"] = wp;

    wp.pose.position.x = 1.8512;
    wp.pose.position.y = -16.0318;
    waypoints_["operating_room"] = wp;

    wp.pose.position.x = -2.9545;
    wp.pose.position.y = -15.5772;
    waypoints_["maternity_room"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));

    progress_ = 0.0;
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Move starting");

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work()
  {
    /*
    if (progress_ < 1.0) {
      progress_ += 0.02;
      send_feedback(progress_, "Move running");
    } else {
      finish(true, 1.0, "Move completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
    */
  }

  float progress_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
