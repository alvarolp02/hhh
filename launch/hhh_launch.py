import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="hhh",
            executable="can_interface_exec",
            name="can_interface",
            output="screen"
        ),
        Node(
            package="hhh",
            executable="dist_control_exec",
            name="can_interface",
            output="screen"
        )
    ])