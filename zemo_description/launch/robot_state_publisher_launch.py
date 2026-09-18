from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(
        get_package_share_directory("zemo_description"),
        "urdf",
        "zemo.urdf"
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": use_sim_time}
            ]
        )
    ])