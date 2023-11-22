#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.action import IncludeLaunchDescription, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    arduino_node = Node(
        package="pruner_lbc",
        executable="arduino_node",
        output="screen"
    )

    ur_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("pruner_lbc"), "ur_startup.launch.py")
        ),
        launch_arguments=[
            ("robot_ip", '169.254.177.50'),
            ("ur_type", 'ur5e'),
            ("use_fake_hardware", "false"),
        ],
    )

    return LaunchDescription([
        arduino_node,
        ur_launch
    ])