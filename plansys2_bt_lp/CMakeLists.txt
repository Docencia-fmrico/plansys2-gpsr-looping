cmake_minimum_required(VERSION 3.5)
project(plansys2_bt_lp)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
)

include_directories(include)

add_library(plansys2_dropitem_bt_node SHARED src/behavior_tree_nodes/DropItem.cpp)
list(APPEND plugin_libs plansys2_dropitem_bt_node)

add_library(plansys2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs plansys2_move_bt_node)

add_library(plansys2_grandmachores_bt_node SHARED src/behavior_tree_nodes/GrandmaChores.cpp)
list(APPEND plugin_libs plansys2_grandmachores_bt_node)

add_library(plansys2_itemforgrandma_bt_node SHARED src/behavior_tree_nodes/ItemForGrandma.cpp)
list(APPEND plugin_libs plansys2_itemforgrandma_bt_node)

add_library(plansys2_movethroughdoor_bt_node SHARED src/behavior_tree_nodes/MoveThroughDoor.cpp)
list(APPEND plugin_libs plansys2_movethroughdoor_bt_node)

add_library(plansys2_opendoor_bt_node SHARED src/behavior_tree_nodes/OpenDoor.cpp)
list(APPEND plugin_libs plansys2_opendoor_bt_node)

add_library(plansys2_organizeitem_bt_node SHARED src/behavior_tree_nodes/OrganizeItem.cpp)
list(APPEND plugin_libs plansys2_organizeitem_bt_node)

add_library(plansys2_pickitem_bt_node SHARED src/behavior_tree_nodes/PickItem.cpp)
list(APPEND plugin_libs plansys2_pickitem_bt_node)

add_library(plansys2_requestopendoor_bt_node SHARED src/behavior_tree_nodes/RequestOpenDoor.cpp)
list(APPEND plugin_libs plansys2_requestopendoor_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(grandma_controller_node src/grandma_controller_node.cpp)
ament_target_dependencies(grandma_controller_node ${dependencies})

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
ament_target_dependencies(nav2_sim_node ${dependencies})

install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  grandma_controller_node
  nav2_sim_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
