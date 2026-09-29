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
  localize on our map (`map_server` + AMCL) and plan and follow a path to the goal.

    * `$ sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander`

* To rebuild the map only (`tools/build_map.py`): `slam_toolbox` and `laser_filters`.

    * `$ sudo apt-get install ros-jazzy-slam-toolbox ros-jazzy-laser-filters`

## Task

This solution (`parc_nav_solution`) navigates with Nav2 on **our own map of
the cafe** (`maps/cafe.yaml`), as the Engineers League coordinator requires
(build your own map and navigate with it). `map_server` loads the map, AMCL
localizes the robot on it with the LiDAR, the global costmap plans on it
(static layer, plus the live LiDAR and a wide margin around obstacles), and
the local costmap follows the robot in its odometry frame.

That odometry frame is `odom_imu` rather than the simulator's `odom`: the
robot's wheel odometry misjudges every turn by ~31%, so `imu_odom_corrector`
combines the IMU's heading with the wheels' distance travelled and publishes
the corrected frame. The frame chain is `map → odom_imu → odom →
base_footprint` (AMCL, `imu_odom_corrector`, the simulator).

The map was built with `tools/build_map.py` (`slam_toolbox` over
`odom_imu`), starting at the robot's spawn pose, so the map's origin is the
spawn pose. The goal coordinates from `parc_robot_bringup`'s
`task_params.yaml` are in Gazebo's world frame; the node transforms them into
the map frame using the inverse of the spawn pose (also in
`task_params.yaml`), gives AMCL the initial pose (0, 0, 0), and sends the goal
to Nav2's `BasicNavigator`.

In local testing with the final configuration (8 runs with the official
`task.launch.py`), every run reached the goal in 57–63 s (median 58.5 s,
including ~16 s for AMCL to confirm the initial pose) without touching any
furniture, driving the same ~10.2 m route and stopping 0.08–0.14 m from the
goal marker's centre (Gazebo ground truth). The 13 runs before that, with
Nav2's stock behavior tree, took 57–84 s and 12 were contact-free; the
exception was a spin recovery that turned the robot into `cafe_table_7`
(see Challenges Faced), which the final configuration's recovery no longer
does. The earlier map-free version (solution 1: 18 of 18 runs without
contact, 50–64 s, ~12 m route) is in the git history.

The controller drives the robot directly; there is deliberately no
`velocity_smoother` (see Challenges Faced).

`ros2 run parc_nav_solution task_solution.py --camera` is an experimental
mode that also feeds the top depth camera into the costmaps (see
`config/nav2_params_camera.yaml`); it isn't the default yet.

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
bags/run_<timestamp>` before playing one back. `tools/score_run.py
<timestamp>` then reports what the robot touched, from the simulator's
contact sensors — a run can report SUCCEEDED and still have pushed a table.

`logs/` and `bags/` are gitignored (regenerated every run) and should be
excluded from the submission zip too — only the `README.md` and
`parc_nav_solution/` package are meant to ship (the map is in the package,
`parc_nav_solution/maps/`). `tools/build_map.py` rebuilds that map (see
Challenges Faced); it also prints a drift check against Gazebo ground truth.
`tools/ros_graph.py` captures the live ROS graph during a run
and renders it to `docs/` (`ros_graph_overview` is the readable summary;
`--from-json docs/ros_graph.json` re-renders without a running stack).

## Challenges Faced

* No map is provided: `task.launch.py` has no `map_server`. A map-free
  configuration (rolling costmaps only) came first; the coordinator then
  clarified that teams must build their own map and navigate with it, hence
  `tools/build_map.py` and the `map_server` + AMCL setup.
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
* Building the map (`tools/build_map.py`) took several fixes:
    * The first SLAM map, made before the `odom_imu` fix, was unusable (walls
      rotated ~40° and drawn twice): scan matching can't absorb a ~30% error
      on every turn.
    * A loop into the cafe's corners got the robot stuck; its wheels kept
      spinning, wheel odometry "drove" it through a wall, and half the map
      was garbage. Even stops along the usual route stranded it between
      tables (the controller doesn't pivot in place). The map is now built
      on exactly the benchmark drive (spawn → goal, one goal), and the tool
      stops at the first failure.
    * `slam_toolbox`'s scan matching made the map worse: a drift check at
      the end of a build put its pose 2.91 m / 17.5° off Gazebo ground truth,
      while `odom_imu` was 0.00 m / 0.5° off, and the cafe came out tilted.
      With scan matching off (each scan placed at the `odom_imu` pose), the
      map is 0.01 m / 0.7° off and axis-aligned.
    * Every scan stamped the robot's own wheels into the map
      (`slam_toolbox`'s `min_laser_range` didn't keep them out): the robot
      appeared parked at the spawn (so Nav2 saw it starting inside an
      obstacle and couldn't plan) and left a trail of dots along the driven
      route, which blocked that corridor, so the planner detoured and clipped
      `cafe_table_1`. The scan is now pre-filtered (`laser_filters`, returns
      under 0.35 m dropped) and the footprint at the spawn is cleared.
    * Without scan matching, stray returns also leave dotted lines of single
      occupied cells across open floor. Two of them, 0.25 m beside the route
      abeam `cafe_table_7`, make NavFn fail to plan from the robot's own cell
      there (8 of 13 runs); the next replan usually succeeds a moment later.
      Freeing all such isolated specks (148 cells) looked like the fix but
      made things worse: some of them were what kept the planner off a gap
      beside `cafe_table_6`, and 5 of 8 runs went through it and hit that
      table. The map is kept as built.
* Nav2's stock behavior tree recovers from repeated planning failures by
  spinning 90° in place, then waiting, then backing up. Among the tables that
  is risky: in one run three quick replanning failures beside `cafe_table_7`
  (the map specks above) triggered the spin, which turned the robot to face
  that table, and it brushed the table on the way out (18 contact messages).
  `behavior_trees/navigate_to_pose_no_motion_recovery.xml` is the stock tree
  with Spin and BackUp removed: recovery clears the costmaps and waits 2 s,
  and replanning from where the robot stands gets it going again.
    * Result: every table is within 0.02–0.15 m of its true position in the
      map. The tables can be pushed in the simulator, so the map assumes the
      static phase 1 environment.
* `BasicNavigator` runs on wall-clock time by default while the rest of the
  stack runs on simulation time: AMCL silently discarded the initial pose
  (stamped ~1.8e9 s ahead of its clock) and never reported a pose, so the
  navigator waited forever. It now uses simulation time and stamps the
  initial pose 0 ("latest").
* With AMCL's default motion noise (`alpha1`–`alpha4` = 0.2), its correction
  wandered up to 0.95 m with 100+ jumps per run, shifting the path under the
  robot, which then weaved (up to 22.7 m driven for a 12 m route).
  `odom_imu` is already accurate, so AMCL now trusts it (`alpha` 0.05) and
  updates less often; its correction stays within ~0.15 m.
* The LiDAR scans ~5 cm above the floor, so of each cafe table it only sees
  the 0.56 m base plate; the 0.913 m tabletop overhangs that by ~0.18 m at
  the height of the robot's upper chassis. The costmap footprint was set to
  the chassis' true width (0.49 m: its URDF collision box is rotated 90°, so
  its "height" is actually its width) and the global costmap's inflation
  widened (`inflation_radius` 1.1 m, `cost_scaling_factor` 1.5) so the
  planned path keeps clear of the tabletops.
* Goal heading: the task only gives a goal position, but Nav2 goals carry
  an orientation, and the controller doesn't pivot in place, so a robot
  arriving at the wrong angle loops around the goal to re-approach. 0.15
  rad circled every time; 0.4 rad was enough on the map-free route but
  still looped on the map route, which arrives at a different angle. The
  yaw tolerance is now ~π (any heading), with 0.1 m in position.
  Tightening the position to 0.07 m (map-free version) brought the robot
  only ~2 cm closer to the marker, within run-to-run spread.
* `velocity_smoother` was silently bypassed: a group-level `SetRemap` of
  `cmd_vel` to the robot's drive topic is matched before each node's own
  `cmd_vel → cmd_vel_nav` rule, so the controller published straight to the
  robot and the smoother listened to its own output topic
  (`tools/ros_graph.py` made this visible: nothing subscribed to
  `/cmd_vel_smoothed`). Wiring it in properly (controller → `cmd_vel_nav` →
  smoother → robot) made things worse: in interleaved GUI runs, 4 of 16 with
  the smoother clipped `cafe_table_6`'s overhanging top, vs 0 of 15 without
  — its acceleration limiting makes the robot lag the controller and cut
  that corner. So the smoother was removed, and the controller and behavior
  server publish straight to `/robot_base_controller/cmd_vel_unstamped`.
* People (tested with the map-free version): the world's three visitors are
  static models, none on the route.
  A test visitor placed standing on the floor in the robot's usual gap was
  avoided (the robot took the other side of `cafe_table_1`), and one that
  appears 3 m ahead mid-run was avoided too, though the robot hesitated
  (~134 s instead of ~55 s). But the world places its standing visitor at
  z = 0.378 while the floor is at ~0.22, so her collision box floats 16 cm
  up — above the LiDAR's scan plane: a visitor placed like that on the route
  is invisible to the LiDAR, and the robot drove into her.
* Camera (`--camera`, experimental, tested with the map-free version): the
  top depth camera sees that visitor
  and the tabletops. The raw cloud is too heavy and full of `inf`s, so
  `depth_obstacles` thins it; the costmaps use a 3D `VoxelLayer` so camera
  rays can clear stale marks without erasing the LiDAR's; and nothing below
  0.6 m is used, because the sim's depth noise lifts empty-floor points up
  to 0.48 m, and a single stray mark on the path stalls the controller. With
  it the robot no longer touches the floating visitor, but only 3 of 4 plain
  GUI runs succeeded (vs 17 of 17 LiDAR-only), and the detour around a
  visitor blocking the usual gap got stuck — hence not the default. The
  camera only publishes ~5 Hz with the GUI launch, and almost nothing
  headless.
* After Nav2 shut down, `ros2 launch` sometimes never returned, which left
  `task_solution.py` hanging after reporting its result; every wait during
  shutdown is now bounded, with a 30 s watchdog as a last resort.
