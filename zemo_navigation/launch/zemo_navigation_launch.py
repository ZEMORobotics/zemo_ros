import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    package_dir = get_package_share_directory("zemo_navigation")

    params_file = os.path.join(
        package_dir,
        "config",
        "nav2_params.yaml"
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
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
        parameters=[configured_params],
        output="screen",
    )

    nodes = LoadComposableNodes(
        target_container="zemo_navigation_container",
        composable_node_descriptions=[
            ComposableNode(
                package="zemo_navigation",
                plugin="zemo_navigation::MoveActionServer",
                name="move",
                parameters=[configured_params],
            ),

            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params],
            ),

            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[configured_params],
            ),

            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params],
            ),

            #ComposableNode(
            #    package="nav2_map_server",
            #    plugin="nav2_map_server::MapServer",
            #    name="map_server",
            #    parameters=[
            #        {
            #            'yaml_filename': map_file,
            #            'topic_name': 'map',
            #            'frame_id': 'map',
            #        }
            #    ],
            #),

            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": [
                            #"map_server",
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