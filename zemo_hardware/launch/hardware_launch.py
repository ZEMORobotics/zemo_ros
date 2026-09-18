import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

def check_can_interface(interface='can0'):
    path = f'/sys/class/net/{interface}'

    if not os.path.exists(path):
        raise RuntimeError(f'[CANBUS ERROR] {interface} does not exist. Is MCP251863/SPI connected?')

    with open(f'{path}/flags', 'r') as f:
        flags = int(f.read().strip(), 16)
    IFF_UP = 0x1

    if not (flags & IFF_UP):
        raise RuntimeError(f'[CANBUS ERROR] {interface} exists but is DOWN. You need to run "canup" to setup CANBUS')

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    log_level = "warn"  # info or warn or error or debug or fatal
    output_screen = "screen" # screen or log or both

    lidar_usb_port = "nesto"

    check_can_interface()

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
        ros_arguments=['--log-level', log_level],
        output=output_screen
    )

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_ros',
        parameters=[{
            'frame_id': 'base_scan',
            'serial_port': lidar_usb_port,
            'serial_baudrate': 115200,
        }],
        ros_arguments=['--log-level', log_level],
        output=output_screen,
    )

    return [
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,

        #lidar,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
    