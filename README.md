# Headless Nav2 navigation

This independent module runs Nav2 on ROS 2 Jazzy / Ubuntu 24.04 using Cyclone
DDS. It supports Raspberry Pi 5 (`linux/arm64`) and x86-64 development hosts.
The containing robot project provides matching Jazzy drivers, localization and
command routing; this module does not start those components.

## Structure

```text
navigation/
  config/navigation.yaml
  src/navigation_bringup/
    launch/navigation.launch.py
    test/
    CMakeLists.txt
    package.xml
  Dockerfile
  compose.yaml
  docker-entrypoint.sh
  navigation.sh
```

`navigation_bringup` starts controller, planner, smoother, behavior, BT navigator,
waypoint follower, velocity smoother and navigation lifecycle manager processes.
The local and global costmaps are owned by the controller and planner. The launch
does not start SLAM, AMCL, map server, sensors, odometry, TF publishers or RViz.
There are no dependencies on other modules in this collection.

## Input and output contracts

| Interface | Default | Type / requirement |
| --- | --- | --- |
| Map | `map` | `nav_msgs/msg/OccupancyGrid`, reliable and transient-local |
| Odometry | `odom` | `nav_msgs/msg/Odometry`, including velocity |
| Obstacle observations | `scan` | `sensor_msgs/msg/LaserScan`, sensor-data QoS |
| TF | `map -> odom -> base_link -> sensor frame` | Connected transforms with consistent timestamps |
| Velocity output | `cmd_vel` | `geometry_msgs/msg/Twist` |
| Single-goal action | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| Multiple-goal action | `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` |
| Waypoint action | `follow_waypoints` | `nav2_msgs/action/FollowWaypoints` |

External localization must provide exactly one publisher of `map -> odom`.
The map must use the configured global frame. Odometry and sensor transforms
must continue updating while navigation runs. Sensor timestamps and TF must use
the same clock as Nav2.

Controller and recovery commands feed `cmd_vel_nav`; the velocity smoother
subscribes to that topic and publishes the configured output, default `cmd_vel`.
The output must differ from `cmd_vel_nav` to avoid a command feedback loop.
All three command-producing/consuming nodes explicitly use unstamped `Twist`.
When changing to `TwistStamped`, set `enable_stamped_cmd_vel` consistently in
`controller_server`, `behavior_server` and `velocity_smoother`.

The module also installs `velocity_mux`, an optional standalone command arbiter:

```bash
ros2 run navigation_bringup velocity_mux --ros-args \
  -r navigation:=/cmd_vel/navigation -r manual:=/cmd_vel/joystick_stamped \
  -r joy:=/joy -r output:=/cmd_vel/raw
```

`navigation` takes physical `Twist`, `manual` takes physical `TwistStamped`,
and `output` publishes physical `TwistStamped`. Input subscriptions use best-effort,
volatile, keep-last 10, accepting both reliable and best-effort publishers. Output
uses reliable, volatile, keep-last 10. Never feed normalized joystick axes here.
With `manual_mode: joystick`, the configured `enable_button` (default 9) grants
manual priority while Joy messages remain fresh, even with centered sticks.
Release or a missing joystick returns control to fresh Nav2 commands. Missing
commands from all sources produce zero. With the button held but stale manual
velocity, output stays zero instead of allowing autonomous movement.
`manual_mode: keyboard` grants priority while moving, then releases after `timeout`;
idle zero messages do not block navigation. `manual_mode: none` selects Nav2 only.
Timeouts use a monotonic clock. `selected_source` reports manual/navigation/stop.
This module's bringup starts the arbiter when `command_mux.ros__parameters.enabled`
is true. The simulation workspace enables it through a temporary config and wires
`/cmd_vel/raw` through its stamped-to-Twist adapter.
The downstream motor layer still owns acceptance, limits and arming, and publishes
`cmd_vel/accepted` for the EKF.

## One configuration file

All settings live in `config/navigation.yaml`. `navigation_bringup.ros__parameters`
contains namespace (empty by default), `use_sim_time`, autostart, log level and
`cmd_vel_topic`. These are launch settings, not a separate running node.
The only launch argument is `config_file`:

```bash
ros2 launch navigation_bringup navigation.launch.py \
  config_file:=/absolute/path/to/navigation.yaml
```

Without that argument, the launch uses the YAML installed in the package share
directory. Compose explicitly selects `/ws/config/navigation.yaml` from the
read-only bind mount. Restart the container after editing the file; no rebuild
is required for YAML changes.

The remaining sections use standard ROS parameter-file syntax. Plugin lists,
plugin class names, tuning, frames, topics, robot footprint, rates and limits
can be changed without editing Python. Unknown plugin parameters are left to
Nav2 to validate. Missing files, malformed YAML and missing required sections
produce errors that identify the configuration path and affected setting.

| Subsystem | Initial plugins |
| --- | --- |
| Global planner | `nav2_smac_planner::SmacPlanner2D` |
| Local controller | `dwb_core::DWBLocalPlanner`, explicit DWB critics |
| Progress / goal checking | `PoseProgressChecker` / `SimpleGoalChecker` |
| Path smoothing | `nav2_smoother::SimpleSmoother` |
| Global costmap | StaticLayer, ObstacleLayer, InflationLayer |
| Local rolling costmap | ObstacleLayer, InflationLayer |
| Recovery behaviors | Spin, BackUp, DriveOnHeading, Wait |
| Waypoint task | WaitAtWaypoint |

The installed Nav2 behavior trees provide replanning and recovery. Optional
custom BT XML paths can be supplied through the navigator's standard parameters.
A different plugin must also be installed in the image and declared in
`package.xml`; changing the YAML does not install software.

The differential-drive configuration uses a 0.10 m radius, 0.15 m/s forward speed,
0.5 rad/s path-following angular speed, 0.5 m/s² linear acceleration, and 1.0 rad/s²
angular acceleration. These are example values, not measured limits. Update both
costmap footprints and keep controller, behavior and velocity-smoother limits
consistent with the actual robot. Distances are meters, angles radians, rates
Hz and time seconds unless marked otherwise in the YAML.

`use_sim_time` in the bringup section overrides the server and nested costmap
clock settings. The launch generates lifecycle `node_names` from its actual
server list and takes autostart from the bringup section. Lifecycle bond tuning
remains in `lifecycle_manager_navigation`.

### Connect the existing SLAM algorithms

For the current RTAB-Map configuration, set
`global_costmap.global_costmap.ros__parameters.static_layer.map_topic` to
`rtabmap/map` when both containers run in the root namespace. RTAB-Map's default
base frame is already `base_link`.

For SLAM Toolbox, the default map topic is `map`, but its example uses
`base_footprint`. Either configure a consistent existing TF chain containing
both base frames or change the Nav2 `robot_base_frame` values consistently in
the navigator, both costmaps and behavior server. Do not introduce a second
publisher for an existing transform.

All topic defaults are relative. With a non-empty bringup namespace, external
publishers should share that namespace or use deliberately configured absolute
input topic names. The launch scopes `tf` and `tf_static` to that namespace too;
external TF publishers must use the matching TF topics. Frame IDs themselves
are not automatically prefixed. A map topic is resolved relative to the parent
Nav2 namespace, not the costmap's private namespace.

## Build and manage

Run from this directory:

```bash
./navigation.sh check
./navigation.sh build
./navigation.sh create
```

The script accepts `build`, `create` (`-c`), `start` (`-s`), `stop` (`-t`),
`restart` (`-r`), `remove` (`-x`), `status`, `logs`, `check` and `help` (`-h`).
`create` builds and starts the service; `start` reuses its existing container.
`remove` removes only the module container. Configuration files remain on disk.
The script resolves its paths relative to itself and can be called from any
working directory.

Compose defaults to `linux/arm64`. On an x86-64 host:

```bash
ROBOT_PLATFORM=linux/amd64 ./navigation.sh build
ROBOT_PLATFORM=linux/amd64 ./navigation.sh create
```

Set the intended domain when creating the service:

```bash
ROS_DOMAIN_ID=7 ./navigation.sh create
```

Changes to environment variables or Compose settings require `create` to
recreate the container; `restart` only reloads files mounted into that container.
The image uses a non-root system user, host networking and no hardware mounts
or privileged access. Build tools and source are confined to the build stage.
Cyclone DDS is selected in the image and Compose; `ROS_DOMAIN_ID` defaults to 0.

## Validation

From the collection root:

```bash
docker compose -f navigation/compose.yaml config --quiet
bash -n navigation/navigation.sh navigation/docker-entrypoint.sh
xmllint --noout navigation/src/navigation_bringup/package.xml
colcon list --base-paths navigation/src
```

In an environment with Jazzy and the declared dependencies installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths navigation/src --packages-select navigation_bringup
source install/setup.bash
colcon test --packages-select navigation_bringup --event-handlers console_direct+
colcon test-result --verbose
ros2 launch navigation_bringup navigation.launch.py --show-args
```

The launch tests check invalid files/settings, ROS YAML parsing, root and nested
namespaces, simulation clock propagation, lifecycle membership and velocity
routing. The routing checks require `nav2_common`.

Run the opt-in synthetic integration test with the installed package sourced:

```bash
NAVIGATION_RUN_SMOKE=1 python3 -m pytest -q \
  navigation/src/navigation_bringup/test/test_navigation_smoke.py
```

It publishes a free map, LaserScan, stationary odometry and TF in local domain
213 under a dedicated namespace, starts the real launch, verifies all seven
servers are active, computes a path, checks `Twist` output and cancels a goal.
It verifies a zero command after cancellation, then shuts down its processes.
It requires the test dependencies declared in `package.xml` and never connects
to motor control. Its use of stationary odometry does not test goal completion.

Build and inspect the native image:

```bash
ROBOT_PLATFORM=linux/amd64 docker compose -f navigation/compose.yaml build
docker run --rm --platform linux/amd64 robot-modules/navigation:jazzy \
  ros2 pkg prefix navigation_bringup
docker run --rm --platform linux/amd64 robot-modules/navigation:jazzy \
  ros2 launch navigation_bringup navigation.launch.py --show-args
```

Build the Raspberry Pi target with the module as context:

```bash
docker buildx build --platform linux/arm64 \
  --file navigation/Dockerfile \
  --tag robot-modules/navigation:jazzy-arm64 navigation
```

Runtime validation must confirm all seven managed servers become active, plugin
loading succeeds, a path can be planned, a navigation goal produces `Twist`,
and canceling that goal stops the command. Use synthetic map, scan, odometry
and TF inputs before connecting motor control. Physical validation must then
check footprint/limits, obstacle response, goal completion and cancellation,
and DDS data delivery between the actual Humble and Jazzy containers. Successful
static checks or image builds do not establish those hardware properties.

Clock selection: set `use_sim_time` in this module’s ROS parameter YAML.
For standalone use, `false` uses the system clock and `true` subscribes to `/clock`.
When launched by Open-PiRobot_simulation, `config/simulation.yaml` →
`runtime.use_sim_time` overrides module YAML values; `null` preserves them.

`command_mux.ros__parameters.enabled` controls the optional arbiter in this module's
bringup. When enabled, the velocity smoother publishes to `navigation_topic` and
the arbiter uses the configured manual, Joy and output topics. It is not a lifecycle
node and is not included in the lifecycle manager's node list. The simulation
workspace enables it through `parameter_overrides`; standalone Nav2 leaves it disabled.
The launch contains ROS actions only; YAML validation is in the package's
`configuration.py`. Costmap `width` and `height` use integer metres as required by Jazzy.

The simulation tuning uses a 0.10 m circular robot radius (zero footprint padding)
and a 0.15 m inflation radius, giving a 0.05 m clearance beyond the robot radius.
Path following uses DWB with zero lateral motion and forward-only linear
velocity sampling. The velocity smoother has no configured deadband.
The navigation smoke test now requires forward velocity >= 0.03 m/s and checks
that cancellation produces zero, instead of accepting any nonzero rotation.
