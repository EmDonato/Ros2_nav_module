#!/usr/bin/env python3
"""Start headless Nav2 using one YAML file and external localization."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from navigation_bringup.configuration import load_configuration


# These are Nav2's standard server nodes. Their planner, controller, behavior,
# costmap and smoother plugins are selected in the module's navigation.yaml.
NAV2_SERVERS = (
    ("controller_server", "nav2_controller", "controller_server"),
    ("smoother_server", "nav2_smoother", "smoother_server"),
    ("planner_server", "nav2_planner", "planner_server"),
    ("behavior_server", "nav2_behaviors", "behavior_server"),
    ("velocity_smoother", "nav2_velocity_smoother", "velocity_smoother"),
    ("bt_navigator", "nav2_bt_navigator", "bt_navigator"),
    ("waypoint_follower", "nav2_waypoint_follower", "waypoint_follower"),
)


def _launch_setup(context):
    path = Path(os.path.expanduser(LaunchConfiguration("config_file").perform(context))).resolve()
    document, bringup = load_configuration(path)
    mux = document['command_mux']['ros__parameters']
    velocity_output = mux['navigation_topic'] if mux['enabled'] else bringup['cmd_vel_topic']

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=str(path),
            root_key=bringup["namespace"],
            param_rewrites={"use_sim_time": str(bringup["use_sim_time"]).lower()},
            convert_types=True,
        )
    )
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = []
    for name, package, executable in NAV2_SERVERS:
        remaps = list(tf_remaps)
        if name in {"controller_server", "behavior_server", "velocity_smoother"}:
            remaps.append(("cmd_vel", "cmd_vel_nav"))
        if name == "velocity_smoother":
            remaps.append(("cmd_vel_smoothed", velocity_output))
        nodes.append(
            Node(
                package=package,
                executable=executable,
                name=name,
                output="screen",
                parameters=[configured_params],
                remappings=remaps,
                arguments=["--ros-args", "--log-level", bringup["log_level"]],
            )
        )
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                configured_params,
                {
                    "use_sim_time": bringup["use_sim_time"],
                    "autostart": bringup["autostart"],
                    "node_names": [name for name, _, _ in NAV2_SERVERS],
                },
            ],
            arguments=["--ros-args", "--log-level", bringup["log_level"]],
        )
    )
    if mux['enabled']:
        nodes.append(Node(
            package='navigation_bringup', executable='velocity_mux', name='command_mux',
            parameters=[configured_params], output='screen',
            remappings=[('joy', mux['joy_topic']), ('manual', mux['manual_topic']),
                        ('navigation', mux['navigation_topic']), ('output', mux['output_topic']),
                        ('selected_source', mux['status_topic'])]))
    return [
        LogInfo(msg=f"Starting Nav2 with external localization using '{path}'"),
        GroupAction(actions=[PushRosNamespace(bringup["namespace"]), *nodes]),
    ]


def generate_launch_description():
    """Expose a single configuration-file argument."""
    default_config = os.path.join(
        get_package_share_directory("navigation_bringup"), "config", "navigation.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the complete navigation YAML configuration",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
