#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Author: Woojin Wie

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments let you override defaults from the command line.
    # e.g. ros2 launch ffw_moveit_config moveit.launch.py use_sim:=true start_rviz:=false
    declared_arguments = [
        # Whether to open RViz with the MoveIt plugin for visualization.
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Whether to execute rviz2'
        ),
        # IMPORTANT: set to true when running with Gazebo.
        # Gazebo publishes its own /clock topic and all nodes must use sim time,
        # otherwise timestamps will mismatch and motion planning will fail.
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Whether to use simulation time',
        ),
        # MoveIt can save/load motion plans to a local SQLite database.
        # Not needed for our use case but required by move_group at startup.
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='Whether to publish robot description semantic',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    # MoveItConfigsBuilder automatically reads all YAMLs from ffw_moveit_config/config/:
    #   ffw.srdf          -> planning groups (arm_r, arm_l), end effectors
    #   kinematics.yaml   -> IK solver (KDL) per group
    #   moveit_controllers.yaml -> which ROS2 controllers MoveIt sends trajectories to
    #   joint_limits.yaml -> max velocity/acceleration per joint
    #   ompl_planning.yaml -> motion planner settings (RRTConnect etc.)
    # You do NOT need to load these manually in your own code.
    moveit_config = (
        MoveItConfigsBuilder(robot_name='ffw', package_name='ffw_moveit_config')
        .robot_description_semantic(Path('config') / 'ffw.srdf')
        .planning_pipelines(pipelines=['ompl', 'stomp', 'pilz_industrial_motion_planner'])
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    # move_group is the MoveIt brain — this is the node your Python code talks to.
    # It receives pose/joint goals, runs IK, plans a collision-free trajectory,
    # and sends it to the appropriate controller (arm_l_controller, arm_r_controller).
    # Your moveit_client.py does NOT talk to the controllers directly —
    # it sends goals to move_group and move_group handles the rest.
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),       # passes all YAML configs to move_group
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,                                        # must match Gazebo when simulating
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
        ],
    )

    # RViz with the MoveIt plugin loaded (moveit.rviz config).
    # Lets you visualize planned paths, set pose goals interactively,
    # and inspect the planning scene (collision objects).
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ffw_moveit_config'), 'config', 'moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,            # URDF (robot geometry)
            moveit_config.robot_description_semantic,   # SRDF (planning groups)
            moveit_config.robot_description_kinematics, # kinematics.yaml (IK solver)
            moveit_config.planning_pipelines,           # planner configs
            moveit_config.joint_limits,                 # joint_limits.yaml
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
            },
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            move_group_node,
            rviz_node,
        ]
    )
