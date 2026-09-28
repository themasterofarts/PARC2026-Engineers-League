"""Sito-É side view with bubble callouts for each onboard sensor (bilingual, 16:9).

    python3 tools/robot_sensors.py [out.png]            # default: docs/robot_sensors.png
    python3 tools/robot_sensors.py --topics [out.png]   # ROS topics per sensor: docs/robot_sensor_topics.png

Anchors come from the URDF joint chain (see robot_render.py, same deps).
Update CALLOUTS when a sensor's role in the solution changes.
"""
import os
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon

from robot_render import side_view

TOPICS = "--topics" in sys.argv
_args = [a for a in sys.argv[1:] if a != "--topics"]
OUT = _args[0] if _args else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs",
                                          "robot_sensor_topics.png" if TOPICS else "robot_sensors.png")
SURFACE, INK, INK2, GRID = "#ffffff", "#0b0b0b", "#52514e", "#e4e3df"
STATUS = {  # colour + text tag: identity is never colour alone
    "used": ("#1a8f5f", "✓ utilisé / used"),
    "camera": ("#c77c0e", "◐ option, désactivée par défaut / off by default"),
    "unused": ("#7a7975", "○ non utilisé / not used"),
}

# Anchor points in metres (base_footprint: x forward, z up) from the URDF joint chain.
LIDAR = (-0.021, 0.048)          # lidar_link + sensor pose 1 cm up
IMU = (0.020, 0.092)
WHEEL = (0.095, 0.086)
TOP_CAM = (-0.035, 1.013)
BOTTOM_CAM = (0.085, 0.583)
CHASSIS = (-0.12, 0.80)

CALLOUTS = [  # (anchor, box centre, status, title, lines) — plain words: this is an intro slide
    (TOP_CAM, (0.78, 1.22), "camera", "Caméra 3D du haut / Top 3D camera",
     ["1,01 m, vers l'avant / 1.01 m, facing forward",
      "Voit plateaux et personnes / sees tabletops, people"]),
    (BOTTOM_CAM, (0.78, 0.74), "unused", "Caméra 3D du bas / Bottom 3D camera",
     ["0,58 m, tournée vers le sol", "0.58 m, facing the floor"]),
    (WHEEL, (0.78, 0.26), "used", "Roues / Wheels",
     ["Distance parcourue / distance travelled"]),
    (CHASSIS, (-0.98, 1.02), "unused", "Capteurs de contact / Contact sensors",
     ["Pas par le robot : servent seulement à noter les essais",
      "Not by the robot: only used to score the runs"]),
    (IMU, (-0.98, 0.56), "used", "IMU (centrale inertielle)",
     ["Direction du robot / robot direction"]),
    (LIDAR, (-0.98, 0.14), "used", "LiDAR (laser)",
     ["À ~5 cm du sol / ~5 cm off the floor",
      "Détecte les obstacles / detects obstacles",
      "Ne voit pas les plateaux / can't see tabletops"]),
]


# Same sensors, bodies listing their ROS topics (from parc_robot_bringup's gz_bridge.yaml / task.launch.py).
TOPIC_CALLOUTS = [
    (TOP_CAM, (0.84, 1.22), "camera", "Caméra 3D du haut / Top 3D camera",
     ["/top_camera_depth/points · PointCloud2",
      "/top_camera_color/image_raw · Image",
      "/top_camera_{color,depth}/camera_info"]),
    (BOTTOM_CAM, (0.84, 0.76), "unused", "Caméra 3D du bas / Bottom 3D camera",
     ["/bottom_camera_depth/points · PointCloud2",
      "/bottom_camera_color/image_raw · Image",
      "/bottom_camera_{color,depth}/camera_info"]),
    (WHEEL, (0.84, 0.31), "used", "Roues / Wheels",
     ["/odom · Odometry",
      "/tf (odom → base_footprint)",
      "/joint_states · JointState",
      "commande / command (Twist) :",
      "/robot_base_controller/cmd_vel_unstamped"]),
    (CHASSIS, (-1.0, 1.06), "unused", "Capteurs de contact / Contact sensors",
     ["/top_chassis_collisions",
      "/base_collisions",
      "/left_wheel_collisions",
      "/right_wheel_collisions",
      "type : ros_gz_interfaces/Contacts"]),
    (IMU, (-1.0, 0.56), "used", "IMU (centrale inertielle)",
     ["/imu · Imu"]),
    (LIDAR, (-1.0, 0.16), "used", "LiDAR (laser)",
     ["/scan · LaserScan"]),
]

def fov(ax, origin, pitch_deg, half_deg, length, color):
    """Shade a camera's vertical field of view (pitch < 0 = looking down)."""
    ox, oz = origin
    pts = [origin]
    for a in (pitch_deg - half_deg, pitch_deg + half_deg):
        pts.append((ox + length * math.cos(math.radians(a)), oz + length * math.sin(math.radians(a))))
    ax.add_patch(Polygon(pts, closed=True, facecolor=color, edgecolor="none", alpha=0.10, zorder=0))


def main():
    fig = plt.figure(figsize=(13.33, 7.5), facecolor=SURFACE)
    ax = fig.add_axes([0, 0.07, 1, 0.93])
    ax.set_facecolor(SURFACE)
    ax.set_xlim(-1.62, 1.42)
    ax.set_ylim(-0.06, 1.44)
    ax.set_aspect("equal")
    ax.axis("off")

    half_vfov = math.degrees(math.atan(math.tan(math.radians(45)) * 480 / 640))  # 90° hfov, 4:3
    fov(ax, TOP_CAM, 0, half_vfov, 0.42, STATUS["camera"][0])
    fov(ax, BOTTOM_CAM, -60, half_vfov, 0.42, STATUS["unused"][0])
    ax.plot([-0.5, 0.52], [0, 0], color=GRID, lw=2, zorder=1)                      # floor
    side_view(ax)
    # LiDAR scan plane: rises 2.5° from the sensor.
    x1 = 0.52
    ax.plot([LIDAR[0], x1], [LIDAR[1], LIDAR[1] + math.tan(math.radians(2.5)) * (x1 - LIDAR[0])],
            color=STATUS["used"][0], lw=1.6, ls=(0, (4, 3)), zorder=6)

    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    to_data = ax.transData.inverted()
    for anchor, (bx, bz), status, title, lines in (TOPIC_CALLOUTS if TOPICS else CALLOUTS):
        color, tag = STATUS[status]
        # Title (bold ink), body (regular), tag (bold, status colour): measure, then stack and box them.
        parts = [(title, dict(fontsize=11.5, fontweight="bold", color=INK)),
                 ("\n".join(lines), dict(fontsize=10.5 if TOPICS else 11.5, color=INK2, linespacing=1.35,
                                          family="DejaVu Sans Mono" if TOPICS else None)),
                 (tag, dict(fontsize=11, fontweight="bold", color=color))]
        texts = [ax.text(bx, bz, t, ha="center", va="center", zorder=12, **kw) for t, kw in parts]
        ext = [t.get_window_extent(renderer) for t in texts]
        (x0, y0), (x1, y1) = to_data.transform([(0, 0), (1, 1)])
        sx, sz = x1 - x0, y1 - y0                      # data units per pixel
        gap = 6 * sz
        heights = [e.height * sz for e in ext]
        width = max(e.width for e in ext) * sx
        total = sum(heights) + gap * (len(heights) - 1)
        top = bz + total / 2
        for t, h in zip(texts, heights):
            t.set_position((bx, top - h / 2))
            top -= h + gap
        pad = 0.035
        bw, bh = width + 2 * pad, total + 2 * pad
        ax.add_patch(FancyBboxPatch((bx - bw / 2, bz - bh / 2), bw, bh, boxstyle="round,pad=0,rounding_size=0.03",
                                    facecolor="#fafaf8", edgecolor=color, linewidth=2, zorder=10))
        # Leader: anchor -> nearest point on the box edge along the line to its centre.
        ax_, az = anchor
        dx, dz = bx - ax_, bz - az
        t_edge = min(abs((bw / 2) / dx) if dx else 1e9, abs((bh / 2) / dz) if dz else 1e9)
        ex, ez = bx - dx * t_edge, bz - dz * t_edge
        ax.plot([ax_, ex], [az, ez], color=color, lw=1.6, zorder=9)
        ax.plot(*anchor, "o", ms=9, mfc=color, mec="white", mew=2, zorder=11)

    footer = ("Topics ROS 2 échangés avec la simulation · ROS 2 topics exchanged with the simulation (ros_gz_bridge, ros_gz_image)"
              if TOPICS else "Vue de côté à l'échelle, d'après le modèle officiel du robot · "
                             "Side view to scale, from the official robot model")
    fig.text(0.5, 0.03, footer,
             ha="center", fontsize=11, color=INK2)
    fig.savefig(OUT, dpi=150, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.12)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
