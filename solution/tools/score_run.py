#!/usr/bin/env python3
"""Score one run_and_log.sh run: result, duration, and what the robot touched.

Usage (inside the container, from /root/ros2_ws/src/solution):

    ros2 bag reindex bags/run_<timestamp>    # run_and_log.sh leaves bags unindexed
    python3 tools/score_run.py <timestamp>

Contacts come from the simulator's contact sensors (/top_chassis_collisions,
/base_collisions, the wheel topics), so they are ground truth: "contacts:
none" means the robot touched nothing but the floor. A run can report
SUCCEEDED and still have pushed a table, so judge tuning changes by contacts
over several runs, not by the result line of one.
"""
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from ros_gz_interfaces.msg import Contacts

SOL = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FLOOR = ("cafe",)  # the cafe model includes the floor, touched all the time


def main():
    ts = sys.argv[1]
    text = open(f"{SOL}/logs/run_{ts}.log").read()
    result = "SUCCEEDED" if "Goal reached" in text else "FAILED" if "Navigation failed" in text else "UNKNOWN"

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=f"{SOL}/bags/run_{ts}", storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    contacts, first, last = {}, None, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        first, last = first or t, t
        if topic.endswith("_collisions"):
            for c in deserialize_message(data, Contacts).contacts:
                other = c.collision2.name.split("::")[0]
                if other not in FLOOR:
                    key = f"{topic.strip('/').replace('_collisions', '')}->{other}"
                    contacts[key] = contacts.get(key, 0) + 1

    span = (last - first) / 1e9 if first else 0
    print(f"run {ts}: {result}  bag span {span:.0f}s  'collision ahead' warnings: {text.count('collision ahead')}")
    print("contacts:", contacts or "none")


if __name__ == "__main__":
    main()
