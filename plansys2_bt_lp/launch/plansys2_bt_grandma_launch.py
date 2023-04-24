# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_bt_lp')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    # Specify the PDDL domain
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/home_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    grandma_chores_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='grandma_chores',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'grandma_chores',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/grandma_chores.xml'
          }
        ])

    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1670,
            'server_port': 1671,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    item_for_grandma_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='item_for_grandma',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'item_for_grandma',
            'publisher_port': 1672,
            'server_port': 1673,
            'bt_xml_file': example_dir + '/behavior_trees_xml/item_for_grandma.xml'
          }
        ])

    organize_item_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='organize_item',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'organize_item',
            'publisher_port': 1674,
            'server_port': 1675,
            'bt_xml_file': example_dir + '/behavior_trees_xml/organize_item.xml'
          }
        ])

    open_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='open_door',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'open_door',
            'publisher_port': 1676,
            'server_port': 1677,
            'bt_xml_file': example_dir + '/behavior_trees_xml/open_door.xml'
          }
        ])

    move_through_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_through_door',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move_through_door',
            'publisher_port': 1678,
            'server_port': 1679,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move_through_door.xml'
          }
        ])

    drop_item_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='drop_item',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'drop_item',
            'publisher_port': 1680,
            'server_port': 1681,
            'bt_xml_file': example_dir + '/behavior_trees_xml/drop_item.xml'
          }
        ])

    pick_item_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick_item',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pick_item',
            'publisher_port': 1682,
            'server_port': 1683,
            'bt_xml_file': example_dir + '/behavior_trees_xml/pick_item.xml'
          }
        ])

    request_open_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='request_open_door',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'request_open_door',
            'publisher_port': 1684,
            'server_port': 1685,
            'bt_xml_file': example_dir + '/behavior_trees_xml/request_open_door.xml'
          }
        ])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(grandma_chores_cmd)
    ld.add_action(move_cmd)

    ld.add_action(item_for_grandma_cmd)
    ld.add_action(organize_item_cmd)
    ld.add_action(open_door_cmd)
    ld.add_action(move_through_door_cmd)
    ld.add_action(drop_item_cmd)
    ld.add_action(pick_item_cmd)
    ld.add_action(request_open_door_cmd)


    return ld
