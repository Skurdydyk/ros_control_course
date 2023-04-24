#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    # Gazebo launch
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo world: configure as needed
    pkg_gazebo_world = get_package_share_directory('spawn_rrbot')
    world_relative_path = 'worlds'
    world_filename = 'empty.world'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    """
    # Robot model: configure as needed (for robot_state_publisher & spawn_entity_client.py)
    robot_model_package = "rrbot_unit5"
    robot_model_relative_path = "urdf"
    robot_model_file = "rrbot.xacro"
    robot_name = "rrbot"
    position_x = "0.0"
    position_y = "0.0"
    position_z = "0.0"

    xacro_file = os.path.join(get_package_share_directory(
        robot_model_package), robot_model_relative_path, robot_model_file)

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(package='spawn_robot',
                        executable='spawn_entity_client.py',
                        arguments=[robot_model_package,
                                   robot_model_relative_path,
                                   robot_model_file,
                                   robot_name,
                                   position_x,
                                   position_y,
                                   position_z],
                        output='screen')

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
    ])
