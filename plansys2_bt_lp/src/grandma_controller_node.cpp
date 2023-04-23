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

class Grandma : public rclcpp::Node
{
public:
  Grandma()
  : rclcpp::Node("grandma_controller")
  {
  }

  // PROBLEM 4
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
    std::cout << "INIT KNOWLEDGE" << std::endl;
    problem_expert_->addInstance(plansys2::Instance{"Kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Bathroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Grandma_Bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Guest_Bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Garage", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Livingroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Yard", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Outside", "room"});

    std::cout << "INIT INSTANCES" << std::endl;

    problem_expert_->addInstance(plansys2::Instance{"Corridor", "corridor"});

    problem_expert_->addInstance(plansys2::Instance{"Door1", "door"});
    problem_expert_->addInstance(plansys2::Instance{"Door2", "door"});
    problem_expert_->addInstance(plansys2::Instance{"Door3", "door"});
    problem_expert_->addInstance(plansys2::Instance{"Door4", "door"});
    problem_expert_->addInstance(plansys2::Instance{"Door5", "door"});
    problem_expert_->addInstance(plansys2::Instance{"Door6", "door"});
    problem_expert_->addInstance(plansys2::Instance{"Door7", "door"});
    problem_expert_->addInstance(plansys2::Instance{"Door8", "door"});

    problem_expert_->addInstance(plansys2::Instance{"Loopy", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"Gm", "grandma"});

    problem_expert_->addInstance(plansys2::Instance{"Towel", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Cutlery", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Tools", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Clothes", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Plants", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Magazines", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Milk", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Pills", "item"});
    problem_expert_->addInstance(plansys2::Instance{"Bed", "item"});

    // Robot
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at Loopy Livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_free Loopy)"));

    // House dimension && connection
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Livingroom Corridor Door3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Kitchen Corridor Door4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Bathroom Livingroom Door2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Yard Corridor Door6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Garage Corridor Door8)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Grandma_Bedroom Corridor Door7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Guest_Bedroom Corridor Door5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Corridor Livingroom Door3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Corridor Kitchen Door4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Livingroom Bathroom Door2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Corridor Yard Door6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Corridor Garage Door8)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Corridor Grandma_Bedroom Door7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Corrior Guest_Bedroom Door5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Outside Livingroom Door1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door Livingroom Outside Door1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(close Door1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close Door2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close Door3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close Door4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close Door5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close Door6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close Door7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close Door8)"));

    problem_expert_->addPredicate(plansys2::Predicate("(grandma_at Gm Grandma_Bedroom)"));

    // Objects to be organized
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Towel Grandma_Bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Cutlery Kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Tools Kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Clothes Yard)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Plants Yard)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Magazines Livingroom)"));

    // Objects used for grandma priorities
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Milk Kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Pills Livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at Bed Guest_Bedroom)"));

    // Organize the following objects
    problem_expert_->addPredicate(plansys2::Predicate("(item_should be Towel Bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_disorganized Towel)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_should_be Tools Garage)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_disorganized Tools)"));

    // Grandma chores
    problem_expert_->addPredicate(plansys2::Predicate("(grandma_get_open_door)"));
    problem_expert_->addPredicate(plansys2::Predicate("(grandma_wants Milk)"));

    // Organize Towel, give Milk grandma
    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(item_organized Towel) (no_grandma_chores))"));
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
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Grandma>();

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
