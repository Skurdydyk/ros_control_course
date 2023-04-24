import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    rviz_config = os.path.join(get_package_share_directory(
        "rrbot_unit5"), "rviz", "unit5.rviz")

    robot_model_package = "rrbot_unit5"
    robot_model_relative_path = "urdf"
    robot_model_file = "rrbot.xacro"

    robot_description = os.path.join(get_package_share_directory(
        robot_model_package), robot_model_relative_path, robot_model_file)

    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            "rrbot_unit5"), "config", "controller_configuration.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller",
                       "-c", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller",
                       "-c", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_velocity_controller",
                       "-c", "/controller_manager"],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output={
                "stdout": "screen",
                "stderr": "log",
            },
        )

    ])
