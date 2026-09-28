"""Orthographic side view of the Sito-É from its URDF + meshes (no simulator needed).

Every link's pose comes from the URDF joint chain, so sensor positions in the
picture are exact. Run directly to print link/sensor positions in
base_footprint (x forward, z up); --preview writes a bare side view.
Needs: pip install trimesh pycollada numpy matplotlib (not in the container).
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

DESC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "parc_robot_description"))
URDF = f"{DESC}/urdf/sitoe_robot.urdf"


def rpy_xyz(el):
    xyz = [float(v) for v in (el.get("xyz") or "0 0 0").split()] if el is not None else [0, 0, 0]
    rpy = [float(v) for v in (el.get("rpy") or "0 0 0").split()] if el is not None else [0, 0, 0]
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, xyz
    return T


def link_poses(root):
    joints = {j.find("child").get("link"): (j.find("parent").get("link"), rpy_xyz(j.find("origin")))
              for j in root.findall("joint")}
    poses = {"base_footprint": np.eye(4)}

    def pose(link):
        if link not in poses:
            parent, T = joints[link]
            poses[link] = pose(parent) @ T
        return poses[link]

    for link in root.findall("link"):
        pose(link.get("name"))
    return poses


def meshes(root, poses):
    """Yield (link name, world-frame trimesh, rgba) for every visual mesh."""
    for link in root.findall("link"):
        for vis in link.findall("visual"):
            m = vis.find("geometry/mesh")
            if m is None:
                continue
            path = m.get("filename").replace("package://parc_robot_description", DESC)
            T = poses[link.get("name")] @ rpy_xyz(vis.find("origin"))
            scene = trimesh.load(path, force="scene")
            for g in scene.dump(concatenate=False) if hasattr(scene, "dump") else [scene]:
                g = g.copy()
                g.apply_transform(T)
                color = np.array([150, 150, 150, 255])
                try:
                    mat = g.visual.material
                    c = getattr(mat, "main_color", None)
                    if c is None and hasattr(mat, "diffuse"):
                        c = mat.diffuse
                    if c is not None:
                        color = np.array(c)
                except Exception:
                    pass
                yield link.get("name"), g, color


def main():
    root = ET.parse(URDF).getroot()
    poses = link_poses(root)
    for name in ("lidar_link", "imu_link", "top_camera_link", "bottom_camera_link", "left_wheel_link",
                 "right_wheel_link", "caster_wheel_link", "top_chassis_link", "base_link"):
        p = poses[name][:3, 3]
        print(f"{name:20s} x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f}")
    n = 0
    for name, g, c in meshes(root, poses):
        b = g.bounds
        print(f"  mesh {name:20s} faces={len(g.faces):6d} color={c[:3]} x[{b[0][0]:+.2f},{b[1][0]:+.2f}] "
              f"y[{b[0][1]:+.2f},{b[1][1]:+.2f}] z[{b[0][2]:+.2f},{b[1][2]:+.2f}]")
        n += len(g.faces)
    print("total faces", n)


if __name__ == "__main__" and "--preview" not in sys.argv:
    main()


def side_view(ax, root=None, poses=None, lighten=True):
    """Draw the robot seen from its left side (+y) onto ax, in metres: x forward (right), z up."""
    from matplotlib.collections import PolyCollection
    root = root if root is not None else ET.parse(URDF).getroot()
    poses = poses if poses is not None else link_poses(root)
    light = np.array([0.35, 0.75, 0.55]); light /= np.linalg.norm(light)
    polys, cols, depth = [], [], []
    for _, g, c in meshes(root, poses):
        normals = g.face_normals
        vis = normals[:, 1] > 0.02                      # faces toward a viewer at +y
        tri = g.triangles[vis]
        if not len(tri):
            continue
        base = np.array(c[:3], float) / 255
        if lighten and base.mean() < 0.25:               # near-black body -> readable grey
            base = np.array([0.42, 0.43, 0.46])
        shade = 0.55 + 0.45 * np.clip(normals[vis] @ light, 0, 1)
        rgb = np.clip(base[None, :] * shade[:, None], 0, 1)
        polys.append(tri[:, :, [0, 2]])
        cols.append(rgb)
        depth.append(tri[:, :, 1].mean(axis=1))
    polys, cols, depth = np.concatenate(polys), np.concatenate(cols), np.concatenate(depth)
    order = np.argsort(depth)                           # far (-y) first, near (+y) last
    ax.add_collection(PolyCollection(polys[order], facecolors=cols[order], edgecolors="none",
                                     antialiased=False, rasterized=True))
    return poses


if __name__ == "__main__" and "--preview" in sys.argv:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(5, 7))
    side_view(ax)
    ax.set_xlim(-0.45, 0.35); ax.set_ylim(-0.02, 1.4); ax.set_aspect("equal")
    ax.axhline(0, color="#8a8984", lw=1)
    fig.savefig("robot_side_preview.png", dpi=150, bbox_inches="tight")
    print("wrote robot_side_preview.png")
