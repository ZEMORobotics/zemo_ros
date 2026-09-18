import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    log_level = "warn"  # info or warn or error or debug or fatal
    output_screen = "screen" # screen or log or both

    controller_config = os.path.join(
        get_package_share_directory('zemo_hardware'),
        'config',
        'controllers.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_config,
            {'use_sim_time': use_sim_time}
        ],
        ros_arguments=['--log-level', log_level],
        output=output_screen
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        ros_arguments=['--log-level', log_level],
        output=output_screen
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controller_config
        ],
        ros_arguments=['--log-level', 'warn'],
        output=output_screen
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])



    