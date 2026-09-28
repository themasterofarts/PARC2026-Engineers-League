#!/usr/bin/env python3
"""PARC 2026 Navigation Task solution entry point.

Brings up Nav2 on our own map of the cafe (see config/nav2_params.yaml and
maps/cafe.yaml, built with tools/build_map.py) and drives the robot to the
competition goal defined in parc_robot_bringup's task_params.yaml.

task_params.yaml gives the robot's spawn pose and the goal position in
Gazebo's WORLD frame. Our map's origin is the spawn pose (slam_toolbox starts
its map frame where the robot starts), so world -> map is just the inverse of
the spawn pose, and the robot's initial pose on the map is (0, 0, 0). AMCL
localizes the robot on the map from there, on top of the IMU-corrected
odometry (odom_imu, see imu_odom_corrector.py).

`--camera` (experimental) adds the top depth camera to the costmaps; see
config/nav2_params_camera.yaml.
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
import rclpy.parameter
import rclpy.time
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def _compose(a, b):
    """SE(2) composition: apply pose b in the frame defined by pose a."""
    ax, ay, ayaw = a
    bx, by, _ = b
    x = ax + math.cos(ayaw) * bx - math.sin(ayaw) * by
    y = ay + math.sin(ayaw) * bx + math.cos(ayaw) * by
    return x, y


def _invert(a):
    x, y, yaw = a
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    return (-(cos_y * x - sin_y * y), -(sin_y * x + cos_y * y), -yaw)


def start_nav2(use_camera=False) -> subprocess.Popen:
    """Launch the Nav2 bringup as a subprocess.

    `launch.LaunchService.run()` registers signal handlers and only works
    in the main thread, so it can't be embedded in-process alongside the
    BasicNavigator logic below — shelling out to `ros2 launch` avoids that
    restriction entirely.

    `start_new_session=True` puts this subprocess (and everything `ros2
    launch` in turn spawns — controller_server, planner_server, etc.) in
    its own process group, so stop_nav2() can signal the whole tree at
    once. Without it, killing just this one PID leaves every node process
    `ros2 launch` spawned running as an orphan.
    """
    return subprocess.Popen(
        ["ros2", "launch", "parc_nav_solution", "nav2_bringup.launch.py",
         f"use_camera:={'true' if use_camera else 'false'}"],
        start_new_session=True,
    )


def stop_nav2(nav2_process: subprocess.Popen) -> None:
    """Tear down the Nav2 launch subprocess.

    Observed in practice: after SIGINT, every lifecycle node exits cleanly
    (each prints its own final "Destroying" line), but the `ros2 launch`
    process itself sometimes never returns from its own shutdown sequence
    even though none of its children are left running — a known class of
    `ros2 launch`/asyncio-loop issue, not something under our control here.
    So every wait below is bounded; if the process is still unreaped after
    SIGKILL there is nothing more we can do to reap it gracefully, and the
    watchdog in main() guarantees this program exits regardless.
    """
    pgid = os.getpgid(nav2_process.pid)
    os.killpg(pgid, signal.SIGINT)
    try:
        nav2_process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        pass

    os.killpg(pgid, signal.SIGKILL)
    try:
        nav2_process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass


def _watchdog_force_exit(timeout_sec: float) -> threading.Timer:
    """Guarantee process exit even if something in shutdown hangs.

    `stop_nav2` bounds its own waits, but `rclpy.shutdown()` and
    `navigator.lifecycleShutdown()` (an rclpy service call under the hood)
    have no such guarantee. Without this, a hang anywhere in shutdown blocks
    `ros2 run parc_nav_solution task_solution.py` forever, which in turn
    blocks `run_and_log.sh`'s foreground `| tee` and it never logs a result.
    """

    def _force_exit():
        print(
            f"[task_solution] shutdown did not complete within {timeout_sec:.0f}s "
            "— forcing process exit",
            flush=True,
        )
        os._exit(1)

    timer = threading.Timer(timeout_sec, _force_exit)
    timer.daemon = True
    timer.start()
    return timer


def load_task_params():
    params_path = os.path.join(
        get_package_share_directory("parc_robot_bringup"), "config", "task_params.yaml"
    )
    with open(params_path) as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]


def main():
    params = load_task_params()
    spawn_pose_world = (params["x"], params["y"], params["yaw"])
    goal_world = (params["goal_x"], params["goal_y"], 0.0)

    world_to_map = _invert(spawn_pose_world)
    goal_x_map, goal_y_map = _compose(world_to_map, goal_world)

    nav2_process = start_nav2(use_camera="--camera" in sys.argv[1:])

    rclpy.init()
    try:
        navigator = BasicNavigator()
        # Run on simulation time like the rest of the stack: BasicNavigator
        # defaults to wall-clock time, and AMCL silently discards an initial
        # pose stamped ~1.8e9 s ahead of its sim clock (it can't look up the
        # robot's motion "since" then), so amcl_pose never arrives.
        navigator.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        navigator.get_logger().info("Waiting for Nav2 to become active...")
        # The robot starts at the map's origin (see the module docstring).
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        # Stamp 0 = "use the latest transform": the robot hasn't moved yet.
        initial_pose.header.stamp = rclpy.time.Time().to_msg()
        initial_pose.pose.orientation.w = 1.0
        navigator.setInitialPose(initial_pose)
        navigator.waitUntilNav2Active(localizer="amcl")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x_map
        goal_pose.pose.position.y = goal_y_map
        goal_pose.pose.orientation.w = 1.0

        navigator.get_logger().info(
            f"Navigating to goal (map frame): ({goal_x_map:.2f}, {goal_y_map:.2f})"
        )
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                navigator.get_logger().info(
                    f"Distance remaining: {feedback.distance_remaining:.2f} m"
                )
            time.sleep(1.0)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info("Goal reached.")
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn("Navigation canceled.")
        else:
            navigator.get_logger().error("Navigation failed.")

        navigator.lifecycleShutdown()
    finally:
        watchdog = _watchdog_force_exit(30)
        rclpy.shutdown()
        stop_nav2(nav2_process)
        watchdog.cancel()


if __name__ == "__main__":
    main()
