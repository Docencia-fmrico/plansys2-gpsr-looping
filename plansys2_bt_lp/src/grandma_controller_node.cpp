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

    // Points near doors - connected with doors
    problem_expert_->addInstance(plansys2::Instance{"p1_living", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p2_living", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p2_bathroom", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p3_corridor", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p3_living", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p4_kitchen", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p4_corridor", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p5_guest", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p5_corridor", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p6_yard", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p6_corridor", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p7_grandma_bed", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p7_corridor", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p8_garage", "near_door"});
    problem_expert_->addInstance(plansys2::Instance{"p8_corridor", "near_door"});

    // Points in rooms - connected without doors
    problem_expert_->addInstance(plansys2::Instance{"spawn_point", "room"});
    problem_expert_->addInstance(plansys2::Instance{"grandma_pos", "room"});
    problem_expert_->addInstance(plansys2::Instance{"towel_bathroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"cutlery_kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"milk_kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"tools_garage", "room"});
    problem_expert_->addInstance(plansys2::Instance{"clothes_bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"plants_yard", "room"});
    problem_expert_->addInstance(plansys2::Instance{"magazines_livingroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"pills_livingroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bed_guestroom", "room"});

    // Problem Coords (Disorganized items)
    problem_expert_->addInstance(plansys2::Instance{"towel_bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"tools_kitchen", "room"});

    // Doors
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

    // Items
    problem_expert_->addInstance(plansys2::Instance{"towel", "item"});
    problem_expert_->addInstance(plansys2::Instance{"cutlery", "item"});
    problem_expert_->addInstance(plansys2::Instance{"tools", "item"});
    problem_expert_->addInstance(plansys2::Instance{"clothes", "item"});
    problem_expert_->addInstance(plansys2::Instance{"plants", "item"});
    problem_expert_->addInstance(plansys2::Instance{"magazines", "item"});
    problem_expert_->addInstance(plansys2::Instance{"milk", "item"});
    problem_expert_->addInstance(plansys2::Instance{"pills", "item"});
    problem_expert_->addInstance(plansys2::Instance{"bed", "item"});

    // Robot
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at loopy spawn_point)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_free loopy)"));

    // Connect near-door points (with doors)
    // Door2
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p2_living p2_bathroom door2)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p2_bathroom p2_living door2)"));
    // Door3
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p3_corridor p3_living door3)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p3_living p3_corridor door3)"));
    // Door4
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p4_corridor p4_kitchen door4)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p4_kitchen p4_corridor door4)"));
    // Door5
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p5_corridor p5_guest door5)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p5_guest p5_corridor door5)"));
    // Door6
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p6_corridor p6_yard door6)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p6_yard p6_corridor door6)"));
    // Door7
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p7_corridor p7_grandma_bed door7)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p7_grandma_bed p7_corridor door7)"));
    // Door8
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p8_corridor p8_garage door8)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_door p8_garage p8_corridor door8)"));

    // Connect room points (without doors)
    // LivingRoom
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p1_living p3_living)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p3_living p1_living)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p3_living pills_livingroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected pills_livingroom p3_living)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected pills_livingroom magazines_livingroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected magazines_livingroom pills_livingroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p3_living p2_living)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p2_living p3_living)"));

    // Bathroom
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p2_bathroom towel_bathroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected towel_bathroom p2_bathroom)"));

    // Corridor
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p3_corridor spawn_point)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected spawn_point p3_corridor)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p4_corridor spawn_point)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected spawn_point p4_corridor)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p5_corridor spawn_point)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected spawn_point p5_corridor)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p6_corridor spawn_point)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected spawn_point p6_corridor)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p6_corridor p7_corridor)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p7_corridor p6_corridor)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p7_corridor p8_corridor)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p8_corridor p7_corridor)"));

    // Guest Bedroom
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p5_guest bed_guestroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected bed_guestroom p5_guest)"));

    // Kitchen
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p4_kitchen milk_kitchen)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected milk_kitchen p4_kitchen)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected milk_kitchen tools_kitchen)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected tools_kitchen milk_kitchen)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected tools_kitchen cutlery_kitchen)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected cutlery_kitchen tools_kitchen)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p4_kitchen cutlery_kitchen)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected cutlery_kitchen p4_kitchen)"));

    // Yard
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p6_yard plants_yard)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected plants_yard p6_yard)"));

    // Grandma Bedroom
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p7_grandma_bed clothes_bedroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected clothes_bedroom p7_grandma_bed)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected clothes_bedroom towel_bedroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected towel_bedroom clothes_bedroom)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected towel_bedroom grandma_pos)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected grandma_pos towel_bedroom)"));

    // Garage
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected p8_garage tools_garage)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected tools_garage p8_garage)"));

    // Init Doors
    problem_expert_->addPredicate(plansys2::Predicate("(close door1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close door8)"));

    problem_expert_->addPredicate(plansys2::Predicate("(grandma_at gm grandma_pos)"));

    // // Objects to be organized
    problem_expert_->addPredicate(plansys2::Predicate("(item_at towel towel_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at cutlery cutlery_kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at tools tools_kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at clothes clothes_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at plants plants_yard)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at magazines magazines_livingroom)"));

    // Objects used for grandma priorities
    problem_expert_->addPredicate(plansys2::Predicate("(item_at milk milk_kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at pills pills_livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_at bed bed_guestroom)"));

    // // Organize the following objects
    problem_expert_->addPredicate(plansys2::Predicate("(item_should_be towel towel_bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_disorganized towel)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_should_be tools tools_garage)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_disorganized tools)"));

    // Grandma chores
    problem_expert_->addPredicate(plansys2::Predicate("(grandma_get_open_door gm)"));
    problem_expert_->addPredicate(plansys2::Predicate("(grandma_has_item gm)"));

    // Organize towel, give milk grandma
    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(item_at towel towel_bathroom))"));
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
