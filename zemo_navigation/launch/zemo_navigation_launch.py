import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    package_dir = get_package_share_directory("zemo_navigation")

    params_file = os.path.join(
        package_dir,
        "config",
        "nav2_params.yaml"
    )

    map_file = os.path.join(
        package_dir,
        "map",
        "map.yaml"
    )

    container = ComposableNodeContainer(
        name="zemo_navigation_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
    )

    nodes = LoadComposableNodes(
        target_container="zemo_navigation_container",
        composable_node_descriptions=[
            ComposableNode(
                package="zemo_navigation",
                plugin="zemo_navigation::MoveActionServer",
                name="move",
                parameters=[params_file],
            ),

            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[params_file],
            ),

            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[params_file],
            ),

            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[params_file],
            ),

            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[
                    {
                        'yaml_filename': map_file,
                        'topic_name': 'map',
                        'frame_id': 'map',
                    }
                ],
            ),

            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": [
                            "map_server",
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                        ],
                    }
                ],
            ),
        ],
    )

    return LaunchDescription([
        container,
        nodes,
    ])