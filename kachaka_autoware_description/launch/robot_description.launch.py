#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("kachaka_autoware_description")
    xacro_path = os.path.join(pkg, "urdf", "kachaka_with_shelf.urdf.xacro")

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace prefix (empty for top-level)",
    )
    frame_prefix_arg = DeclareLaunchArgument(
        "frame_prefix",
        default_value="",
        description="Prefix prepended to every TF frame_id",
    )
    # Real Kachaka deployment publishes dynamic joints (docking_link, wheels)
    # via kachaka_grpc_ros2_bridge's dynamic_tf_bridge_component. For
    # standalone development/visualization without a robot, set to true to
    # run joint_state_publisher with all joints at zero.
    use_jsp_arg = DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="false",
        description="Run joint_state_publisher for offline visualization",
    )

    robot_description = {
        "robot_description": Command(["xacro ", xacro_path]),
        "frame_prefix": LaunchConfiguration("frame_prefix"),
    }

    return LaunchDescription(
        [
            namespace_arg,
            frame_prefix_arg,
            use_jsp_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=LaunchConfiguration("namespace"),
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                namespace=LaunchConfiguration("namespace"),
                condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
                output="screen",
            ),
        ]
    )
