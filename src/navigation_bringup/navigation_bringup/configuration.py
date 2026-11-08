"""Validate the module YAML independently of ROS launch actions."""
import re
import yaml

REQUIRED_SECTIONS = (
    "controller_server", "smoother_server", "planner_server", "behavior_server",
    "velocity_smoother", "bt_navigator", "waypoint_follower",
)


def _parameters(document, *keys):
    section = document
    for key in (*keys, "ros__parameters"):
        section = section.get(key) if isinstance(section, dict) else None
    if not isinstance(section, dict):
        raise ValueError(f"Missing mapping '{'.'.join((*keys, 'ros__parameters'))}'")
    return section


def load_configuration(path):
    """Validate bringup settings without restricting upstream plugin parameters."""
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("The document must contain a YAML mapping")
        bringup = _parameters(document, "navigation_bringup")
        for name in REQUIRED_SECTIONS:
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
