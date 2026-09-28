# TEAM: PARC Engineers League 2026

## Introduction

The 2026 Navigation Task requires an autonomous solution that guides the
CAYTU Sito-É from its spawn position to a goal marker inside a simulated
restaurant, avoiding static furniture and simulated crowd models along the
way, within a 10-minute limit.

**Team Country:** TBD

**Team Member Names:**

* TBD (Team Leader)

## Dependencies

* `nav2` / `nav2-bringup` / `nav2-simple-commander`: navigation stack used to
  plan and follow a path to the goal.

    * `$ sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander`

## Task

This solution (`parc_nav_solution`) brings up a **mapless** Nav2 stack —
no `map_server`/AMCL, since `task.launch.py` provides no static map. Both
costmaps run as rolling windows in an odometry frame, fed by `/scan` for
obstacle detection. That frame is `odom_imu` rather than the simulator's
`odom`: the robot's wheel odometry misjudges every turn by ~30%, so
`imu_odom_corrector` combines the IMU's heading with the wheels' distance
travelled and publishes the corrected frame (see Challenges Faced).

The goal coordinates from `parc_robot_bringup`'s `task_params.yaml` are
given in Gazebo's world frame, so the node transforms them into the
odometry frame using the inverse of the robot's known spawn pose (also in
`task_params.yaml`) — confirmed empirically that the DiffDrive plugin
initializes `odom` at identity relative to the robot's actual spawn pose,
the standard wheel-odometry convention, and `odom_imu` coincides with
`odom` until the robot moves — then sends the transformed goal to Nav2's
`BasicNavigator`.

In local testing (11 consecutive runs, 3 headless and 8 with the official
`task.launch.py`), every run reached the goal in 50–64 s (median 55 s) with
no contact between the robot and any furniture, stopping ~0.12 m from the
goal marker's centre (measured against Gazebo ground truth).

Known issue: `velocity_smoother` is currently bypassed. In
`launch/nav2_bringup.launch.py` the group-level `cmd_vel` remap is matched
before each node's own `cmd_vel → cmd_vel_nav` remap, so the controller
publishes straight to the robot. The runs above were all made this way.

Run with:

```
ros2 run parc_nav_solution task_solution.py
```

(after `ros2 launch parc_robot_bringup task.launch.py` is already running
in another terminal).

## Local benchmarking

`run_and_log.sh` runs one full attempt end to end and records it:

```
./run_and_log.sh          # headless Gazebo (no GUI) — fast, for iterating on tuning
./run_and_log.sh --gui    # official task.launch.py, with Gazebo/RViz windows
```

Each run launches the sim, waits for the robot to spawn, records a rosbag
(`/odom`, `/scan`, `/tf`, `/tf_static`, `cmd_vel`, the four collision
topics) to `bags/run_<timestamp>/`, tees `task_solution.py`'s output to
`logs/run_<timestamp>.log`, and appends a one-line result + elapsed time to
`run_history.md`. `task_headless.launch.py` (used for the default headless
mode) is our own dev-only variant of the official `task.launch.py` —
Gazebo server-only, no RViz — the actual submission always targets the
unmodified official launch file.

`ros2 bag record` doesn't exit on SIGINT here, so the script force-stops it
and the bag is left without its index — run `ros2 bag reindex
bags/run_<timestamp>` before playing one back.

`logs/` and `bags/` are gitignored (regenerated every run) and should be
excluded from the submission zip too — only the `README.md` and
`parc_nav_solution/` package are meant to ship. `tools/build_map.py` (build
a SLAM map of the cafe) is a dev-only experiment the current solution
doesn't use. `tools/ros_graph.py` captures the live ROS graph during a run
and renders it to `docs/` (`ros_graph_overview` is the readable summary;
`--from-json docs/ros_graph.json` re-renders without a running stack).

## Challenges Faced

* No static map is provided, so a mapless costmap-only Nav2 configuration
  was used instead of the usual map_server + AMCL setup.
* The `/sitoe_robot/pose` ground-truth topic is advertised but never
  actually publishes in this world — an earlier version of this node used
  it to calibrate the world→odom transform at runtime; that was dropped in
  favor of computing the transform directly from the known spawn pose in
  `task_params.yaml`, verified against a live `/odom` reading instead.
* Jazzy's stock `nav2_bringup` `navigation_launch.py` brings up several
  servers (`collision_monitor`, `route_server`, `docking_server`,
  `smoother_server`) that aren't configured here and some of which crash
  without extra required parameters — `launch/nav2_bringup.launch.py`
  defines a trimmed node set instead of including that launch file.
* The default `rmw_fastrtps_cpp` pulls in a `nav2_msgs` typesupport
  library that's ABI-incompatible with the `fastcdr` build currently
  shipped for Jazzy (symbol lookup error in `controller_server`) — worked
  around by switching to `rmw_cyclonedds_cpp` (set in the Dockerfile).
* The RPLIDAR sees parts of the robot's own chassis at several angles
  (front/rear mount ~0.05-0.11m, wheels/sides ~0.17-0.29m), saturating the
  costmap with bogus close obstacles and stalling the controller
  (`RegulatedPurePursuitController` reporting "collision ahead"
  indefinitely) — fixed with `obstacle_min_range: 0.35` on the scan
  source.
* The first tuning pass (~6:42 to complete) turned out to be
  controller-tuning-bound, not robot-speed-bound: commanding `cmd_vel`
  linear.x=1.0 directly achieved a clean, unclipped 1.0 m/s in `/odom`.
  The actual bottlenecks were `desired_linear_vel` set far below that, a
  `velocity_smoother` cap of 0.5 m/s clipping the controller's output
  again after the fact, and `use_rotate_to_heading` stopping the robot to
  pivot in place on large heading errors (visible as ~10s of no distance
  progress at the very start of a run) instead of arcing continuously.
* The wheel odometry overestimates every turn by ~30%: in a spin the
  simulator's `odom` reported 134.6° while the robot (per Gazebo ground
  truth) turned 102.5°. Straight-line distance is accurate to ~1%, and the
  IMU's orientation matched ground truth exactly. Before this was found,
  obstacles seen before a turn landed in the wrong place in the costmap
  after it, so the robot repeatedly drove its upper chassis into
  `cafe_table_6`, took a different route on every run, and stopped away
  from the goal marker. `imu_odom_corrector` dead-reckons with the IMU's
  heading and the wheels' forward distance and publishes the correction as
  `odom_imu → odom` (leaving the simulator's `odom → base_footprint`
  untouched); all Nav2 frames use `odom_imu`. Checked against ground truth
  after a spin–drive–spin–drive sequence: `odom` was off by 0.66 m, while
  `odom_imu` was within 2 cm and 0°.
* A SLAM map (`slam_toolbox`, via `tools/build_map.py`) was tried as a way
  to make the route consistent from run to run. It came out unusable —
  walls rotated ~40° and drawn twice — because of the odometry drift above:
  scan matching can't absorb a ~30% error on every turn. It hasn't been
  retried since the `odom_imu` fix: the mapless solution already succeeds
  consistently, and the cafe tables are movable in the simulator (the robot
  pushed one during tuning), so a map recorded ahead of time could go stale.
* The LiDAR scans ~4 cm above the floor, so of each cafe table it only sees
  the 0.56 m base plate; the 0.913 m tabletop overhangs that by ~0.18 m at
  the height of the robot's upper chassis. The costmap footprint was set to
  the chassis' true width (0.49 m: its URDF collision box is rotated 90°, so
  its "height" is actually its width) and the global costmap's inflation
  widened (`inflation_radius` 1.1 m, `cost_scaling_factor` 1.5) so the
  planned path keeps clear of the tabletops.
* A 0.15 rad yaw goal tolerance made the robot circle the goal
  indefinitely (it reached the position repeatedly but never settled on the
  heading while avoiding a nearby table); the tolerance is 0.1 m / 0.4 rad.
* After Nav2 shut down, `ros2 launch` sometimes never returned, which left
  `task_solution.py` hanging after reporting its result; every wait during
  shutdown is now bounded, with a 30 s watchdog as a last resort.
