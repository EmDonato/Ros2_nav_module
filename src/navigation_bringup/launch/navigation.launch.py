#!/usr/bin/env python3
"""Start headless Nav2 using one YAML file and external localization."""

import os
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile


SERVERS = (
    ("controller_server", "nav2_controller", "controller_server"),
    ("smoother_server", "nav2_smoother", "smoother_server"),
    ("planner_server", "nav2_planner", "planner_server"),
    ("behavior_server", "nav2_behaviors", "behavior_server"),
    ("velocity_smoother", "nav2_velocity_smoother", "velocity_smoother"),
    ("bt_navigator", "nav2_bt_navigator", "bt_navigator"),
    ("waypoint_follower", "nav2_waypoint_follower", "waypoint_follower"),
)


def _parameters(document, *keys):
    section = document
    for key in (*keys, "ros__parameters"):
        section = section.get(key) if isinstance(section, dict) else None
    if not isinstance(section, dict):
        raise ValueError(f"Missing mapping '{'.'.join((*keys, 'ros__parameters'))}'")
    return section


def _load_configuration(path):
    """Validate bringup settings without restricting upstream plugin parameters."""
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("The document must contain a YAML mapping")
        bringup = _parameters(document, "navigation_bringup")
        for name, _, _ in SERVERS:
            _parameters(document, name)
        _parameters(document, "lifecycle_manager_navigation")
        for name in ("local_costmap", "global_costmap"):
            _parameters(document, name, name)

        for name in ("use_sim_time", "autostart"):
            if not isinstance(bringup.get(name), bool):
                raise ValueError(f"navigation_bringup.{name} must be a YAML boolean")
        namespace = bringup.get("namespace")
        if not isinstance(namespace, str):
            raise ValueError("navigation_bringup.namespace must be a string")
        namespace = namespace.strip("/")
        if namespace and not re.fullmatch(
            r"[A-Za-z_]\w*(/[A-Za-z_]\w*)*", namespace, re.ASCII
        ):
            raise ValueError(f"Invalid navigation_bringup.namespace '{namespace}'")
        level = bringup.get("log_level")
        if not isinstance(level, str) or level not in {"debug", "info", "warn", "error", "fatal"}:
            raise ValueError(
                "navigation_bringup.log_level must be debug, info, warn, error or fatal"
            )
        topic = bringup.get("cmd_vel_topic")
        if not isinstance(topic, str) or not topic.strip():
            raise ValueError("navigation_bringup.cmd_vel_topic must be a non-empty string")
        internal_topic = f"/{namespace}/cmd_vel_nav" if namespace else "/cmd_vel_nav"
        parent = f"/{namespace}" if namespace else ""
        resolved_topic = topic if topic.startswith("/") else f"{parent}/{topic}"
        if topic == "cmd_vel_nav" or resolved_topic == internal_topic:
            raise ValueError(
                "navigation_bringup.cmd_vel_topic must not feed back into cmd_vel_nav"
            )
    except (OSError, UnicodeError, yaml.YAMLError, ValueError) as error:
        raise RuntimeError(f"Invalid navigation configuration '{path}': {error}") from error
    return document, {**bringup, "namespace": namespace}


def _launch_setup(context):
    path = Path(os.path.expanduser(LaunchConfiguration("config_file").perform(context))).resolve()
    _, bringup = _load_configuration(path)
    from nav2_common.launch import RewrittenYaml

    # Rewrites reach the nested costmap nodes as well as the top-level servers.
    rewrites = {"use_sim_time": str(bringup["use_sim_time"]).lower()}
    for name, _, _ in SERVERS:
        rewrites[f"{name}.ros__parameters.use_sim_time"] = rewrites["use_sim_time"]
    for name in ("local_costmap", "global_costmap"):
        rewrites[f"{name}.{name}.ros__parameters.use_sim_time"] = rewrites["use_sim_time"]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=str(path),
            root_key=bringup["namespace"],
            param_rewrites=rewrites,
            convert_types=True,
        )
    )
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = []
    for name, package, executable in SERVERS:
        remaps = list(tf_remaps)
        if name in {"controller_server", "behavior_server", "velocity_smoother"}:
            remaps.append(("cmd_vel", "cmd_vel_nav"))
        if name == "velocity_smoother":
            remaps.append(("cmd_vel_smoothed", bringup["cmd_vel_topic"]))
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
                    "node_names": [name for name, _, _ in SERVERS],
                },
            ],
            arguments=["--ros-args", "--log-level", bringup["log_level"]],
        )
    )
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
