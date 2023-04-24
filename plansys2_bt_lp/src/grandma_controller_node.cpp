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
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"grandma_bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"guest_bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"garage", "room"});
    problem_expert_->addInstance(plansys2::Instance{"livingroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"yard", "room"});
    problem_expert_->addInstance(plansys2::Instance{"outside", "room"});

    problem_expert_->addInstance(plansys2::Instance{"corridor", "corridor"});

    problem_expert_->addInstance(plansys2::Instance{"door1", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door2", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door3", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door4", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door5", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door6", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door7", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door8", "door"});

    problem_expert_->addInstance(plansys2::Instance{"loopy", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"gm", "grandma"});

    problem_expert_->addInstance(plansys2::Instance{"towel", "item"});
    problem_expert_->addInstance(plansys2::Instance{"cutlery", "item"});
    problem_expert_->addInstance(plansys2::Instance{"tools", "item"});
    problem_expert_->addInstance(plansys2::Instance{"clothes", "item"});
    problem_expert_->addInstance(plansys2::Instance{"plants", "item"});
    problem_expert_->addInstance(plansys2::Instance{"magazines", "item"});
    problem_expert_->addInstance(plansys2::Instance{"milk", "item"});
    problem_expert_->addInstance(plansys2::Instance{"pills", "item"});
    problem_expert_->addInstance(plansys2::Instance{"bed", "item"});

    // // Robot
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at loopy livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_free loopy)"));

    // // House dimension && connection
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door livingroom corridor door3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door kitchen corridor door4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door bathroom livingroom door2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door yard corridor door6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door garage corridor door8)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door grandma_bedroom corridor door7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door guest_bedroom corridor door5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door corridor livingroom door3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door corridor kitchen door4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door livingroom bathroom door2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door corridor yard door6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door corridor garage door8)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door corridor grandma_bedroom door7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door corridor guest_bedroom door5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door outside livingroom door1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_door livingroom outside door1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(close door1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door8)"));

    problem_expert_->addPredicate(plansys2::Predicate("(grandma_at gm grandma_bedroom)"));

    // // Objects to be organized
    problem_expert_->addPredicate(plansys2::Predicate("(item_at towel grandma_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at cutlery kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at tools kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at clothes yard)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at plants yard)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at magazines livingroom)"));

    // Objects used for grandma priorities
    problem_expert_->addPredicate(plansys2::Predicate("(item_at milk kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at pills livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at bed guest_bedroom)"));

    // // Organize the following objects
    problem_expert_->addPredicate(plansys2::Predicate("(item_should_be towel bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_disorganized towel)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_should_be tools garage)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_disorganized tools)"));

    // // Grandma chores
    //problem_expert_->addPredicate(plansys2::Predicate("(grandma_get_open_door)"));
    problem_expert_->addPredicate(plansys2::Predicate("(grandma_wants milk)"));

    // Organize towel, give milk grandma
    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(item_at cutlery livingroom))"));
    std::cout << "END KNOWLEDGE" << std::endl;
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
