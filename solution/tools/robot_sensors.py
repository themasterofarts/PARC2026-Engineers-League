"""Sito-É side view with bubble callouts for each onboard sensor (bilingual, 16:9).

    python3 tools/robot_sensors.py [out.png]     # default: docs/robot_sensors.png

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

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "robot_sensors.png")
SURFACE, INK, INK2, GRID = "#ffffff", "#0b0b0b", "#52514e", "#e4e3df"
STATUS = {  # colour + text tag: identity is never colour alone
    "used": ("#1a8f5f", "✓ utilisé / used"),
    "camera": ("#c77c0e", "◐ mode --camera"),
    "unused": ("#7a7975", "○ non utilisé / not used"),
}

# Anchor points in metres (base_footprint: x forward, z up) from the URDF joint chain.
LIDAR = (-0.021, 0.048)          # lidar_link + sensor pose 1 cm up
IMU = (0.020, 0.092)
WHEEL = (0.095, 0.086)
TOP_CAM = (-0.035, 1.013)
BOTTOM_CAM = (0.085, 0.583)
CHASSIS = (-0.12, 0.80)

CALLOUTS = [  # (anchor, box centre, status, title, lines)
    (TOP_CAM, (0.78, 1.22), "camera", "RealSense D435 (haut / top)",
     ["1,01 m, vers l'avant / 1.01 m, facing forward",
      "Voit plateaux et personnes / sees tabletops, people"]),
    (BOTTOM_CAM, (0.78, 0.74), "unused", "RealSense D435 (bas / bottom)",
     ["0,58 m, incliné 60° vers le sol", "0.58 m, tilted 60° down"]),
    (WHEEL, (0.78, 0.26), "used", "Roues / Wheels (odométrie / odometry)",
     ["Distance parcourue / distance travelled",
      "Virages surestimés de 31 % / turns +31 %"]),
    (CHASSIS, (-0.98, 1.02), "unused", "Capteurs de contact / Contact sensors",
     ["Châssis, base, roues / chassis, base, wheels",
      "Score des essais seulement / run scoring only"]),
    (IMU, (-0.98, 0.56), "used", "IMU",
     ["Direction du robot → repère odom_imu",
      "Robot heading → odom_imu frame"]),
    (LIDAR, (-0.98, 0.14), "used", "LiDAR RPLIDAR C1",
     ["~5 cm du sol, 2,5° vers le haut / ~5 cm up, 2.5° up",
      "Obstacles → costmaps ; aveugle aux plateaux",
      "Obstacles → costmaps; blind to tabletops"]),
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
    for anchor, (bx, bz), status, title, lines in CALLOUTS:
        color, tag = STATUS[status]
        # Title (bold ink), body (regular), tag (bold, status colour): measure, then stack and box them.
        parts = [(title, dict(fontsize=11.5, fontweight="bold", color=INK)),
                 ("\n".join(lines), dict(fontsize=11.5, color=INK2, linespacing=1.35)),
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

    fig.text(0.5, 0.03, "Vue de côté à l'échelle, positions issues de l'URDF · "
             "Side view to scale, positions from the URDF (parc_robot_description)",
             ha="center", fontsize=11, color=INK2)
    fig.savefig(OUT, dpi=150, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.12)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
