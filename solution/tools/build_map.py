#!/usr/bin/env python3
"""Build the cafe occupancy map shipped with parc_nav_solution.

Run inside the container (after `colcon build`):

    python3 /root/ros2_ws/src/solution/tools/build_map.py

Starts the headless sim, slam_toolbox (tools/slam_mapping.yaml) and the
mapless Nav2 stack, drives a loop of waypoints around the cafe, then saves
the map to parc_nav_solution/maps/cafe.{yaml,pgm}. Rebuild the package
afterwards so the new map gets installed.

The cafe tables are dynamic models the robot can push, so every waypoint is
kept >= ~2m from every table: a table shoved during mapping would end up in
the map at the wrong place.

Not used by the current (mapless) solution. Its only run predates the
odom_imu heading fix and produced an unusable map (walls rotated and
duplicated), so inspect the result before relying on it.
"""
import math
import os
import signal
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from parc_nav_solution.task_solution import _compose, _invert, load_task_params

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_OUT = os.path.normpath(os.path.join(HERE, "..", "parc_nav_solution", "maps", "cafe"))
SLAM_PARAMS = os.path.join(HERE, "slam_mapping.yaml")

# World-frame (x, y) waypoints: a loop through the south, east, north and west
# of the cafe, ending near the task goal.
WAYPOINTS_WORLD = [
    (2.5, -8.5),
    (3.0, -3.5),
    (1.3, -1.2),
    (3.0, 3.0),
    (-1.0, 4.5),
    (-3.6, 3.5),
    (-4.0, -2.5),
    (-3.6, -7.5),
    (-2.3, 2.2),
]
WAYPOINT_TIMEOUT_S = 150.0
OVERALL_TIMEOUT_S = 1800.0


def start(name, cmd):
    log = open(f"/tmp/build_map_{name}.log", "w")
    return subprocess.Popen(cmd, start_new_session=True, stdout=log, stderr=subprocess.STDOUT)


def stop(proc):
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    os.killpg(pgid, signal.SIGINT)
    try:
        proc.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        pass
    os.killpg(pgid, signal.SIGKILL)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass


def wait_for_topic(topic, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        out = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True).stdout
        if topic in out.split():
            return True
        time.sleep(1.0)
    return False


def main():
    params = load_task_params()
    world_to_odom = _invert((params["x"], params["y"], params["yaw"]))
    waypoints = [_compose(world_to_odom, (x, y, 0.0)) for x, y in WAYPOINTS_WORLD]

    procs = []
    watchdog = threading.Timer(OVERALL_TIMEOUT_S, lambda: os._exit(1))
    watchdog.daemon = True
    watchdog.start()
    try:
        procs.append(start("sim", ["ros2", "launch", "parc_nav_solution", "task_headless.launch.py"]))
        if not wait_for_topic("/odom", 120):
            raise RuntimeError("sim never published /odom")
        time.sleep(2.0)
        procs.append(start("slam", [
            "ros2", "launch", "slam_toolbox", "online_async_launch.py",
            f"slam_params_file:={SLAM_PARAMS}", "use_sim_time:=true",
        ]))
        procs.append(start("nav2", ["ros2", "launch", "parc_nav_solution", "nav2_bringup.launch.py"]))

        rclpy.init()
        nav = BasicNavigator()
        nav.waitUntilNav2Active(localizer="robot_localization")

        prev = (0.0, 0.0)
        for i, (x, y) in enumerate(waypoints, 1):
            yaw = math.atan2(y - prev[1], x - prev[0])
            prev = (x, y)
            goal = PoseStamped()
            goal.header.frame_id = "odom_imu"
            goal.header.stamp = nav.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            nav.goToPose(goal)
            started = time.time()
            while not nav.isTaskComplete():
                if time.time() - started > WAYPOINT_TIMEOUT_S:
                    nav.cancelTask()
                    break
                time.sleep(0.5)
            result = nav.getResult()
            nav.get_logger().info(
                f"waypoint {i}/{len(waypoints)} {WAYPOINTS_WORLD[i - 1]}: {result.name} "
                f"({time.time() - started:.0f}s)"
            )

        time.sleep(5.0)  # let slam_toolbox publish a final map update
        saved = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", MAP_OUT,
             "--ros-args", "-p", "use_sim_time:=true", "-p", "save_map_timeout:=20.0"],
            capture_output=True, text=True, timeout=60,
        )
        print(saved.stdout[-800:], saved.stderr[-800:])
        print(f"map saved to {MAP_OUT}.yaml" if saved.returncode == 0 else "MAP SAVE FAILED")
        rclpy.shutdown()
    finally:
        for proc in reversed(procs):
            stop(proc)
        watchdog.cancel()


if __name__ == "__main__":
    main()
