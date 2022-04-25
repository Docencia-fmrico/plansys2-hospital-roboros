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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TiagoController : public rclcpp::Node
{
public:
  TiagoController()
  : rclcpp::Node("tiago_controller"), state_(STARTING)
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return true;
  }
  void init_knowledge()
  {
    // Le paso al problem_expert_ las instancias y los predicados
    problem_expert_->addInstance(plansys2::Instance{"high_dependency_room_1", "location"});
    problem_expert_->addInstance(plansys2::Instance{"high_dependency_room_4", "location"});
    problem_expert_->addInstance(plansys2::Instance{"corridor", "door"});
    problem_expert_->addInstance(plansys2::Instance{"robot", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"object1", "object"});

    problem_expert_->addPredicate(
      plansys2::Predicate("(door_joins corridor high_dependency_room_1 high_dependency_room_4)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(door_joins corridor high_dependency_room_4 high_dependency_room_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(opened_door corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at robot high_dependency_room_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at object1 high_dependency_room_1)"));

    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(carry_object robot object1))"));

  }
  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }
private:
  typedef enum {STARTING, MOVING, FINISHED} StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TiagoController>();

  if (!node->init()) {
    return 0;
  }
  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
