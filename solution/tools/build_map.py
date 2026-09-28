#!/usr/bin/env python3
"""Build the cafe occupancy map shipped with parc_nav_solution.

Run inside the container (after `colcon build`):

    python3 /root/ros2_ws/src/solution/tools/build_map.py

Starts the headless sim, slam_toolbox (tools/slam_mapping.yaml) and Nav2 in
mapping mode (no map yet: rolling costmaps in odom_imu), drives a loop of waypoints around the cafe, then saves
the map to parc_nav_solution/maps/cafe.{yaml,pgm}. Rebuild the package
afterwards so the new map gets installed.

The route is exactly the benchmark drive (spawn -> task goal, one goal: 0
table contacts in 18 runs); the LiDAR's 12 m range covers the walls from that
corridor. Two things went wrong with multi-waypoint loops: a wider loop into
the corners got the robot stuck (its wheels kept spinning, wheel odometry
"drove" it through a wall, and half the map came out as garbage), and even
intermediate stops along the benchmark route stranded it between tables (the
controller doesn't pivot in place, so turning toward the next waypoint arcs
into a table's margin). So: one goal, and driving stops at the first failure;
inspect the map before relying on it.
"""
import math
import os
import signal
import subprocess
import threading
import time

import re

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
from nav2_simple_commander.robot_navigator import BasicNavigator

from parc_nav_solution.task_solution import _compose, _invert, load_task_params

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_OUT = os.path.normpath(os.path.join(HERE, "..", "parc_nav_solution", "maps", "cafe"))
SLAM_PARAMS = os.path.join(HERE, "slam_mapping.yaml")
SCAN_FILTER = os.path.join(HERE, "scan_filter.yaml")

# World-frame (x, y) waypoints: just the task goal, i.e. the benchmark drive.
WAYPOINTS_WORLD = [
    (-2.29, 2.23),   # task goal
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


def gz_ground_truth():
    """Robot's true (x, y, yaw) in Gazebo's world frame (dev check only; the robot never uses this)."""
    out = subprocess.run(["gz", "topic", "-e", "-t", "/world/task/pose/info", "-n", "1"],
                         capture_output=True, text=True, timeout=15).stdout
    block = out[out.index('name: "sitoe_robot"'):]
    block = block[:block.index("}\n}") + 3]
    num = lambda k, t: float(re.search(rf"{k}: (-?[\d.e+-]+)", t).group(1)) if re.search(rf"{k}: ", t) else 0.0
    pos, ori = block[block.index("position"):block.index("orientation")], block[block.index("orientation"):]
    qx, qy, qz, qw = (num(k, ori) for k in "xyzw")
    return num("x", pos), num("y", pos), math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def tf_pose(buf, frame):
    t = buf.lookup_transform(frame, "base_footprint", Time(), timeout=Duration(seconds=2.0))
    q = t.transform.rotation
    return (t.transform.translation.x, t.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def report_drift(nav, buf, world_to_odom):
    """Compare SLAM (map) and odom_imu with ground truth, all expressed in the spawn frame."""
    gx, gy, gyaw = gz_ground_truth()
    tx, ty = _compose(world_to_odom, (gx, gy, 0.0))
    tyaw = gyaw + world_to_odom[2]
    for frame in ("map", "odom_imu"):
        try:
            x, y, yaw = tf_pose(buf, frame)
        except Exception as e:  # noqa: BLE001 - diagnostic only
            nav.get_logger().warn(f"{frame}: no transform ({e})")
            continue
        dyaw = math.degrees(math.atan2(math.sin(yaw - tyaw), math.cos(yaw - tyaw)))
        nav.get_logger().info(f"drift check {frame:8s}: pos error {math.hypot(x - tx, y - ty):.2f} m, "
                              f"heading error {dyaw:+.1f} deg (truth ({tx:.2f}, {ty:.2f}))")


def clear_spawn(map_base, radius=0.40):
    """Mark the robot's own footprint at the map origin as free space.

    The first scans include the robot's own wheels/chassis despite
    min_laser_range, so the saved map shows the robot parked at the spawn;
    Nav2 then sees the robot starting inside an obstacle and can't plan. That
    spot is known to be free (the robot stood there), so clear it.
    Returns the number of occupied cells cleared.
    """
    meta = dict(line.split(": ", 1) for line in open(map_base + ".yaml").read().splitlines() if ": " in line)
    res = float(meta["resolution"])
    ox, oy = (float(v) for v in meta["origin"].strip("[]").split(",")[:2])
    raw = open(map_base + ".pgm", "rb").read()
    fields, pos = [], 0
    while len(fields) < 4:  # P5 header: magic, width, height, maxval (comments skipped)
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            pos = raw.index(b"\n", pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        fields.append(raw[pos:end]); pos = end
    width, height = int(fields[1]), int(fields[2])
    header, pixels = raw[:pos + 1], bytearray(raw[pos + 1:])
    cleared = 0
    for row in range(height):
        y = oy + (height - 1 - row + 0.5) * res
        for col in range(width):
            x = ox + (col + 0.5) * res
            if x * x + y * y <= radius * radius:
                i = row * width + col
                cleared += pixels[i] <= 50
                pixels[i] = 254
    open(map_base + ".pgm", "wb").write(header + bytes(pixels))
    return cleared


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
        # /scan -> /scan_filtered without the robot's own chassis/wheel returns.
        procs.append(start("scan_filter", [
            "ros2", "run", "laser_filters", "scan_to_scan_filter_chain",
            "--ros-args", "-r", "__node:=scan_filter", "-r", "scan:=/scan", "-r", "scan_filtered:=/scan_filtered",
            "--params-file", SCAN_FILTER,
        ]))
        procs.append(start("slam", [
            "ros2", "launch", "slam_toolbox", "online_async_launch.py",
            f"slam_params_file:={SLAM_PARAMS}", "use_sim_time:=true",
        ]))
        procs.append(start("nav2", ["ros2", "launch", "parc_nav_solution", "nav2_bringup.launch.py",
                                   "mode:=mapping"]))

        rclpy.init()
        nav = BasicNavigator()
        buf = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(buf, nav, spin_thread=True)  # nav is only spun inside its own calls
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
            if result.name != "SUCCEEDED":
                nav.get_logger().error("stopping: a stuck robot's wheel odometry corrupts the map")
                break

        time.sleep(8.0)  # let slam_toolbox publish a final map update
        report_drift(nav, buf, world_to_odom)
        saved = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", MAP_OUT,
             "--ros-args", "-p", "use_sim_time:=true", "-p", "save_map_timeout:=20.0"],
            capture_output=True, text=True, timeout=60,
        )
        print(saved.stdout[-800:], saved.stderr[-800:])
        print(f"map saved to {MAP_OUT}.yaml" if saved.returncode == 0 else "MAP SAVE FAILED")
        if saved.returncode == 0:
            print(f"cleared {clear_spawn(MAP_OUT)} occupied cells of the robot's own footprint at the spawn")
        del tf_listener  # stop its spin thread before rclpy shuts down (else a traceback on exit)
        rclpy.shutdown()
    finally:
        for proc in reversed(procs):
            stop(proc)
        watchdog.cancel()


if __name__ == "__main__":
    main()
