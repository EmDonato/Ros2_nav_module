"""Opt-in Nav2 integration check using a stationary synthetic robot."""

import math
import os
import signal
import subprocess
import time

import pytest
import yaml


@pytest.mark.skipif(
    os.environ.get("NAVIGATION_RUN_SMOKE") != "1",
    reason="Set NAVIGATION_RUN_SMOKE=1 with Nav2 Jazzy installed to run integration checks",
)
def test_synthetic_navigation(tmp_path, monkeypatch):
    """Activate plugins, compute a path, produce Twist and stop after canceling."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    from test_navigation_launch import CONFIG, navigation

    # Keep synthetic commands and TF in a dedicated local domain and namespace.
    monkeypatch.setenv("ROS_DOMAIN_ID", "213")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    namespace = f"navigation_smoke_{os.getpid()}"
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    document["navigation_bringup"]["ros__parameters"].update(
        namespace=namespace, cmd_vel_topic="commands/nav"
    )
    config = tmp_path / "navigation.yaml"
    config.write_text(yaml.safe_dump(document), encoding="utf-8")

    context = Context()
    rclpy.init(context=context)
    node = Node(
        "synthetic_inputs", context=context, namespace=namespace,
        cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
    )
    log_path = tmp_path / "bringup.log"
    process = None
    try:
        map_pub = node.create_publisher(
            OccupancyGrid, "map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        scan_pub = node.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        odom_pub = node.create_publisher(Odometry, "odom", 10)
        static_tf = StaticTransformBroadcaster(node)
        dynamic_tf = TransformBroadcaster(node)

        def transform(parent, child):
            msg = TransformStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.rotation.w = 1.0
            return msg

        static_tf.sendTransform([transform("map", "odom"), transform("base_link", "laser")])
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = node.get_clock().now().to_msg()
        grid.info.resolution = 0.05
        grid.info.width = 200
        grid.info.height = 200
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 40000
        map_pub.publish(grid)

        def publish_inputs():
            now = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            dynamic_tf.sendTransform(transform("odom", "base_link"))
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = "laser"
            scan.angle_min = -math.pi
            scan.angle_increment = 2.0 * math.pi / 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min = 0.05
            scan.range_max = 5.0
            scan.scan_time = 0.05
            scan.ranges = [4.0] * 360
            scan_pub.publish(scan)

        node.create_timer(0.05, publish_inputs)
        commands = []
        node.create_subscription(
            Twist, "commands/nav", lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10
        )
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                [
                    "ros2", "launch", "navigation_bringup", "navigation.launch.py",
                    f"config_file:={config}",
                ],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )

            def wait_for(predicate, description, timeout=30.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text(encoding="utf-8")
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if predicate():
                        return
                pytest.fail(
                    f"Timed out waiting for {description}\n{log_path.read_text(encoding='utf-8')}"
                )

            clients = {
                name: node.create_client(GetState, f"{name}/get_state")
                for name, _, _ in navigation.SERVERS
            }
            requests = {}

            def all_active():
                ready = True
                for name, client in clients.items():
                    if not client.service_is_ready():
                        ready = False
                        continue
                    future = requests.get(name)
                    if future is None:
                        requests[name] = client.call_async(GetState.Request())
                        ready = False
                    elif not future.done():
                        ready = False
                    elif future.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
                        requests[name] = client.call_async(GetState.Request())
                        ready = False
                return ready

            wait_for(all_active, "all managed servers to become active", timeout=90.0)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = node.get_clock().now().to_msg()
            goal_pose.pose.position.x = 1.0
            goal_pose.pose.orientation.w = 1.0
            planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
            wait_for(planner.server_is_ready, "planner action")
            plan_goal = ComputePathToPose.Goal()
            plan_goal.goal = goal_pose
            plan_goal.planner_id = "GridBased"
            plan_request = planner.send_goal_async(plan_goal)
            wait_for(plan_request.done, "planner goal acceptance")
            assert plan_request.result().accepted
            plan_result = plan_request.result().get_result_async()
            wait_for(plan_result.done, "planned path")
            assert plan_result.result().status == GoalStatus.STATUS_SUCCEEDED
            assert len(plan_result.result().result.path.poses) > 1

            navigator = ActionClient(node, NavigateToPose, "navigate_to_pose")
            wait_for(navigator.server_is_ready, "navigator action")
            goal = NavigateToPose.Goal()
            goal.pose = goal_pose
            request = navigator.send_goal_async(goal)
            wait_for(request.done, "navigation goal acceptance")
            handle = request.result()
            assert handle.accepted
            wait_for(
                lambda: any(abs(v) > 0.001 for command in commands for v in command), "Twist output"
            )
            publishers = node.get_publishers_info_by_topic(f"/{namespace}/commands/nav")
            assert publishers and all(p.topic_type == "geometry_msgs/msg/Twist" for p in publishers)
            cancel_index = len(commands)
            cancellation = handle.cancel_goal_async()
            wait_for(cancellation.done, "goal cancellation")
            assert cancellation.result().goals_canceling
            result = handle.get_result_async()
            wait_for(result.done, "canceled action result")
            assert result.result().status == GoalStatus.STATUS_CANCELED
            wait_for(
                lambda: any(abs(x) < 0.001 and abs(z) < 0.001 for x, z in commands[cancel_index:]),
                "zero velocity after cancellation",
            )
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        node.destroy_node()
        context.shutdown()
