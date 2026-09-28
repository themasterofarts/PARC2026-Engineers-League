#!/usr/bin/env python3
"""PARC 2026 Navigation Task solution entry point.

Brings up a mapless Nav2 stack (see config/nav2_params.yaml) and drives the
robot to the competition goal defined in parc_robot_bringup's
task_params.yaml.

task_params.yaml gives the robot's spawn pose and the goal position in
Gazebo's WORLD frame, but Nav2 here has no map — both costmaps run in the
"odom_imu" frame instead (IMU-heading-corrected odometry; see
imu_odom_corrector.py). odom_imu coincides with odom until the robot moves,
so the goal computed below is valid in either. Empirically (checked via
`ros2 topic echo /odom --once` against the running sim), the DiffDrive
plugin initializes odom at identity — (0, 0, yaw=0) — relative to the
robot's actual spawn pose, which is the standard wheel-odometry convention:
odom always starts at the robot's own body frame, with no knowledge of true
world orientation. So the world->odom transform is just the inverse of the
known spawn pose, and the goal can be computed directly from
task_params.yaml with no live calibration step needed.

(An earlier version of this node calibrated that transform at runtime from
the `/sitoe_robot/pose` ground-truth topic instead — dropped after
confirming that topic is advertised but never actually publishes in this
world.)

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

    world_to_odom = _invert(spawn_pose_world)
    goal_x_odom, goal_y_odom = _compose(world_to_odom, goal_world)

    nav2_process = start_nav2(use_camera="--camera" in sys.argv[1:])

    rclpy.init()
    try:
        navigator = BasicNavigator()
        navigator.get_logger().info("Waiting for Nav2 to become active...")
        # 'robot_localization' is the sentinel value nav2_simple_commander uses
        # to skip waiting on a localizer lifecycle node entirely (see its
        # source: only this exact string bypasses both the activation and
        # initial-pose checks) — we don't run AMCL or robot_localization here.
        navigator.waitUntilNav2Active(localizer="robot_localization")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "odom_imu"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x_odom
        goal_pose.pose.position.y = goal_y_odom
        goal_pose.pose.orientation.w = 1.0

        navigator.get_logger().info(
            f"Navigating to goal (odom frame): ({goal_x_odom:.2f}, {goal_y_odom:.2f})"
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
