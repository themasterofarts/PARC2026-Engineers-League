#!/usr/bin/env python3
"""Capture the live ROS 2 graph (nodes, topics, actions, message types) and render it.

Run inside the container while the stack is up, e.g. during a benchmark run:

    python3 /root/ros2_ws/src/solution/tools/ros_graph.py --wait-for-node basic_navigator

Writes to docs/ by default:
  ros_graph.json            raw capture: every node's publishers, subscribers,
                            action clients and servers, with types
  ros_graph.{dot,svg,png}   data flow: topics with both a publisher and a
                            subscriber, plus actions
  ros_graph_full.{dot,svg,png}  every topic, including ones nobody subscribes to
  ros_graph_overview.{dot,svg,png}  only the main pipeline (OVERVIEW), sized
                            to stay readable on a slide

--from-json re-renders from an existing ros_graph.json without a live stack.
"""
import argparse
import json
import os
import shutil
import subprocess
import time

import rclpy
from rclpy.action import (get_action_client_names_and_types_by_node,
                          get_action_server_names_and_types_by_node)
from rclpy.node import Node

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, "..", "docs"))

# ROS plumbing every node has; drawing it would bury the actual data flow.
IGNORED_TOPICS = {"/rosout", "/parameter_events", "/clock"}
# Implementation-detail nodes (tf2's internal listener nodes) and the benchmark
# harness's recorder, which isn't part of the solution or the official launch.
EXCLUDED_NODE_PREFIXES = ("/transform_listener_impl_", "/rosbag2_recorder", "/ros_graph_capture")

GROUPS = {
    "Notre solution / Our solution": ("#d6e6f9", "#2a78d6"),
    "Nav2": ("#d3f0e4", "#1baf7a"),
    "Simulateur / Simulator": ("#fbe0d4", "#eb6834"),
    "Autre / Other": ("#eeeeec", "#8a8984"),
}
OURS = {"/basic_navigator", "/imu_odom_corrector"}
NAV2_NAMES = ("controller_server", "planner_server", "behavior_server", "bt_navigator",
              "waypoint_follower", "velocity_smoother", "lifecycle_manager", "costmap")
SIM_PREFIXES = ("/ros_gz", "/robot_state_publisher", "/top_camera", "/bottom_camera")

# Sensors in, goal -> plan -> velocity commands out. /cmd_vel_smoothed is kept
# even though nothing subscribes to it: it shows the velocity_smoother is bypassed.
OVERVIEW = {"/scan", "/odom", "/imu", "/tf", "/joint_states", "/robot_base_controller/cmd_vel_unstamped",
            "/cmd_vel_smoothed", "/navigate_to_pose", "/compute_path_to_pose", "/follow_path"}


def group_of(node):
    if node in OURS:
        return "Notre solution / Our solution"
    if any(n in node for n in NAV2_NAMES):
        return "Nav2"
    if node.startswith(SIM_PREFIXES):
        return "Simulateur / Simulator"
    return "Autre / Other"


def hidden(topic):
    return any(part.startswith("_") for part in topic.split("/") if part)


def capture(node):
    def keep(pairs):
        return sorted({t: types[0] for t, types in pairs if t not in IGNORED_TOPICS and not hidden(t)}.items())

    graph = {}
    for name, ns in node.get_node_names_and_namespaces():
        fq = (ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name
        if name.startswith("_"):
            continue
        graph[fq] = {
            "publishes": keep(node.get_publisher_names_and_types_by_node(name, ns)),
            "subscribes": keep(node.get_subscriber_names_and_types_by_node(name, ns)),
            "action_clients": sorted((a, t[0]) for a, t in get_action_client_names_and_types_by_node(node, name, ns)),
            "action_servers": sorted((a, t[0]) for a, t in get_action_server_names_and_types_by_node(node, name, ns)),
        }
    return graph


def to_dot(graph, flow_only, keep=None):
    """keep: if given, draw only these topics/actions (and the nodes on them)."""
    q = lambda s: '"' + s.replace('"', '\\"') + '"'
    graph = {n: i for n, i in graph.items() if not n.startswith(EXCLUDED_NODE_PREFIXES)}

    pubs, subs, types = {}, {}, {}
    clients, servers, action_types = {}, {}, {}
    for n, info in graph.items():
        for t, typ in info["publishes"]:
            pubs.setdefault(t, []).append(n); types[t] = typ
        for t, typ in info["subscribes"]:
            subs.setdefault(t, []).append(n); types[t] = typ
        for a, typ in info["action_clients"]:
            clients.setdefault(a, []).append(n); action_types[a] = typ
        for a, typ in info["action_servers"]:
            servers.setdefault(a, []).append(n); action_types[a] = typ

    if keep:
        topics = [t for t in types if t in keep]
        actions = [a for a in action_types if a in keep]
    else:
        topics = [t for t in types if not flow_only or (t in pubs and t in subs)]
        actions = [a for a in action_types if not flow_only or (a in clients and a in servers)]
    used = {n for t in topics for n in pubs.get(t, []) + subs.get(t, [])}
    used |= {n for a in actions for n in clients.get(a, []) + servers.get(a, [])}
    nodes = sorted(used if flow_only or keep else graph)

    if keep:
        view = "vue d'ensemble : chaîne principale / overview: main pipeline"
    elif flow_only:
        view = ("flux de données : topics avec publieur et abonné, + actions / "
                "data flow: topics with a publisher and a subscriber, + actions")
    else:
        view = "complet / full: tous les topics / every topic"
    font = 16 if keep else 11
    hidden_note = "" if keep else (
        '\\nMasqués / hidden: /rosout, /parameter_events, /clock (tous les nœuds / every node), '
        'transform_listener_impl_*, rosbag2_recorder (banc de test / test harness)')
    lines = [
        "digraph ros_graph {",
        # The overview is laid out top to bottom so it fits a 16:9 slide.
        f"  rankdir={'TB' if keep else 'LR'}; splines=true; nodesep=0.3; ranksep={0.5 if keep else 1.2};",
        f'  node [fontname="Helvetica", fontsize={font}]; edge [color="#8a8984", arrowsize=0.6];',
        '  labelloc=t; fontname="Helvetica"; fontsize=16;',
        f'  label="Graphe ROS 2 / ROS 2 graph — {view}\\n'
        f'Ellipses = nœuds / nodes · rectangles = topics (type) · hexagones = actions (tirets / dashed){hidden_note}";',
    ]
    for n in nodes:
        fill, border = GROUPS[group_of(n)]
        lines.append(f"  {q(n)} [shape=ellipse, style=filled, fillcolor={q(fill)}, color={q(border)}, penwidth=2];")
    for t in sorted(topics):
        lines.append(f"  {q(t)} [shape=box, style=rounded, fontsize={font - 1}, label={q(t + chr(10) + types[t])}];")
    for a in sorted(actions):
        lines.append(f"  {q('action:' + a)} [shape=hexagon, style=dashed, fontsize={font - 1}, "
                     f"label={q(a + chr(10) + action_types[a])}];")
    for t in sorted(topics):
        lines += [f"  {q(n)} -> {q(t)};" for n in pubs.get(t, [])]
        lines += [f"  {q(t)} -> {q(n)};" for n in subs.get(t, [])]
    for a in sorted(actions):
        lines += [f"  {q(n)} -> {q('action:' + a)} [style=dashed];" for n in clients.get(a, [])]
        lines += [f"  {q('action:' + a)} -> {q(n)} [style=dashed];" for n in servers.get(a, [])]
    lines.append('  subgraph cluster_legend { label="Légende / Legend"; fontsize=12; style=dashed; color="#8a8984";')
    for i, (g, (fill, border)) in enumerate(GROUPS.items()):
        lines.append(f"    legend{i} [label={q(g)}, shape=ellipse, style=filled, fillcolor={q(fill)}, "
                     f"color={q(border)}, penwidth=2];")
    lines += ["  }", "}"]
    return "\n".join(lines), len(nodes), len(topics), len(actions)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", default=DEFAULT_OUT)
    p.add_argument("--wait-for-node", help="wait until this node name exists before capturing")
    p.add_argument("--settle", type=float, default=15.0, help="seconds to wait after that node appears")
    p.add_argument("--timeout", type=float, default=300.0)
    p.add_argument("--from-json", help="render this saved capture instead of the live graph")
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    if args.from_json:
        with open(args.from_json) as f:
            graph = json.load(f)
    else:
        graph = capture_live(args)
        with open(os.path.join(args.out_dir, "ros_graph.json"), "w") as f:
            json.dump(graph, f, indent=1)
    for name, flow_only, keep in (("ros_graph", True, None), ("ros_graph_full", False, None),
                                  ("ros_graph_overview", True, OVERVIEW)):
        dot, n_nodes, n_topics, n_actions = to_dot(graph, flow_only, keep)
        base = os.path.join(args.out_dir, name)
        with open(base + ".dot", "w") as f:
            f.write(dot)
        if shutil.which("dot"):
            for fmt in ("svg", "png"):
                subprocess.run(["dot", f"-T{fmt}", "-Gdpi=110", base + ".dot", "-o", f"{base}.{fmt}"], check=True)
        print(f"{name}: {n_nodes} nodes, {n_topics} topics, {n_actions} actions")


def capture_live(args):
    rclpy.init()
    node = Node("ros_graph_capture")
    deadline = time.time() + args.timeout
    if args.wait_for_node:
        while args.wait_for_node not in [n for n, _ in node.get_node_names_and_namespaces()]:
            if time.time() > deadline:
                raise SystemExit(f"node {args.wait_for_node!r} never appeared")
            rclpy.spin_once(node, timeout_sec=0.5)
    end = time.time() + args.settle
    while time.time() < end:  # let graph discovery catch up
        rclpy.spin_once(node, timeout_sec=0.2)
    graph = capture(node)
    node.destroy_node()
    rclpy.shutdown()
    return graph


if __name__ == "__main__":
    main()
