import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zemo_description'),
                'launch',
                'robot_state_publisher_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zemo_hardware'),
                'launch',
                'hardware_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    return [
        robot_state_publisher,
        hardware,

    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
