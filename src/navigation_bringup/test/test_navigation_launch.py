"""Check configuration failures and ROS parameter routing without starting nodes."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch_ros.utilities import to_parameters_list


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT.parents[1] / "config" / "navigation.yaml"
SPEC = importlib.util.spec_from_file_location(
    "navigation_launch", PACKAGE_ROOT / "launch" / "navigation.launch.py"
)
navigation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(navigation)


@pytest.fixture(autouse=True)
def local_launch_logs(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))


@pytest.fixture
def document():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def write_config(tmp_path, document):
    path = tmp_path / "navigation.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def ros_parameters(path, node_name):
    namespace, name = node_name.rsplit("/", 1)
    return {
        parameter.name: parameter.value
        for parameter in to_parameters_list(LaunchContext(), name, namespace, [Path(path)])
    }


def test_default_config_is_accepted_by_ros():
    navigation._load_configuration(CONFIG)
    params = ros_parameters(CONFIG, "/controller_server")
    assert params["enable_stamped_cmd_vel"] is False
    assert params["FollowPath.plugin"] == "dwb_core::DWBLocalPlanner"
    params = ros_parameters(CONFIG, "/global_costmap/global_costmap")
    assert params["static_layer.map_topic"] == "map"


@pytest.mark.parametrize("content", ["[", "[]", "null", "controller_server: false"])
def test_invalid_yaml_reports_path(tmp_path, content):
    path = tmp_path / "invalid.yaml"
    path.write_text(content, encoding="utf-8")
    with pytest.raises(RuntimeError, match=str(path)):
        navigation._load_configuration(path)


def test_missing_file_reports_path(tmp_path):
    path = tmp_path / "missing.yaml"
    with pytest.raises(RuntimeError, match=str(path)):
        navigation._load_configuration(path)


@pytest.mark.parametrize(
    "section", ["planner_server", "local_costmap", "lifecycle_manager_navigation"]
)
def test_missing_sections_report_name(tmp_path, document, section):
    del document[section]
    with pytest.raises(RuntimeError, match=section):
        navigation._load_configuration(write_config(tmp_path, document))


@pytest.mark.parametrize(
    "key,value",
    [
        ("autostart", "false"),
        ("use_sim_time", 1),
        ("namespace", "robot-name"),
        ("namespace", None),
        ("cmd_vel_topic", ""),
        ("cmd_vel_topic", "cmd_vel_nav"),
        ("cmd_vel_topic", "/cmd_vel_nav"),
        ("log_level", "verbose"),
        ("log_level", []),
    ],
)
def test_invalid_bringup_settings(tmp_path, document, key, value):
    document["navigation_bringup"]["ros__parameters"][key] = value
    with pytest.raises(RuntimeError, match=key):
        navigation._load_configuration(write_config(tmp_path, document))


@pytest.mark.parametrize("namespace", ["", "robot/navigation"])
def test_namespace_clock_lifecycle_and_velocity_routing(tmp_path, document, monkeypatch, namespace):
    pytest.importorskip("nav2_common", reason="Nav2 must be installed for launch routing checks")
    bringup = document["navigation_bringup"]["ros__parameters"]
    bringup.update(
        namespace=namespace, use_sim_time=True, autostart=False, cmd_vel_topic="commands/nav"
    )
    # A changed plugin parameter must reach ROS intact, including nested costmaps.
    document["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] = 0.42
    context = LaunchContext()
    context.launch_configurations["config_file"] = str(write_config(tmp_path, document))
    recorded = {}
    original_node = navigation.Node

    def record_node(**kwargs):
        recorded[kwargs["name"]] = kwargs
        return original_node(**kwargs)

    monkeypatch.setattr(navigation, "Node", record_node)
    navigation._launch_setup(context)
    assert len(recorded) == 8
    for name in ("controller_server", "behavior_server", "velocity_smoother"):
        assert ("cmd_vel", "cmd_vel_nav") in recorded[name]["remappings"]
    assert ("cmd_vel_smoothed", "commands/nav") in recorded["velocity_smoother"]["remappings"]
    lifecycle = recorded["lifecycle_manager_navigation"]["parameters"][-1]
    assert lifecycle["autostart"] is False
    assert set(lifecycle["node_names"]) == set(recorded) - {"lifecycle_manager_navigation"}

    parameter_file = recorded["controller_server"]["parameters"][0].evaluate(context)
    prefix = f"/{namespace}" if namespace else ""
    names = [
        *lifecycle["node_names"], "local_costmap/local_costmap", "global_costmap/global_costmap"
    ]
    for name in names:
        params = ros_parameters(parameter_file, f"{prefix}/{name}")
        assert params["use_sim_time"] is True
    params = ros_parameters(parameter_file, f"{prefix}/planner_server")
    assert params["GridBased.tolerance"] == 0.42
