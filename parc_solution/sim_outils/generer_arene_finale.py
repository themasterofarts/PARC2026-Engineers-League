"""Genere arena_finale_world.sdf : arene des photos du 4 nov, tags tires au hasard."""
import math, random, re, sys
SRC = sys.argv[1]; DST = sys.argv[2]
DESK, CHAIR = (0.60, 0.45), (0.50, 0.50)
desks = [((-2.15, 0.90), math.pi / 2), ((2.15, 1.20), math.pi / 2)]   # le long des murets gauche et droit
chairs = [((0.60, -1.20), 0.4), ((0.00, 0.05), -0.3), ((-0.60, 1.30), 0.8)]  # diagonale au milieu
SPAWN = (-1.2, -1.9)

def rect_dist(p, c, yaw, size):
    dx, dy = p[0] - c[0], p[1] - c[1]
    lx = dx * math.cos(-yaw) - dy * math.sin(-yaw); ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    ex, ey = max(abs(lx) - size[0] / 2, 0), max(abs(ly) - size[1] / 2, 0)
    return math.hypot(ex, ey)

def obstacle_dist(p, poles):
    d = min(2.5 - abs(p[0]), 2.5 - abs(p[1]))
    for c, yaw in desks: d = min(d, rect_dist(p, c, yaw, DESK))
    for c, yaw in chairs: d = min(d, rect_dist(p, c, yaw, CHAIR))
    for q in poles: d = min(d, math.dist(p, q))
    return d

for seed in range(1, 100000):
    rng = random.Random(seed); poles, tags, ok = [], [], True
    for tag_id in (1, 4, 7, 9):
        for _ in range(800):
            p = (rng.uniform(-2.3, 2.3), rng.uniform(-2.3, 2.3)); yaw = rng.uniform(-math.pi, math.pi)
            n = (math.cos(yaw), math.sin(yaw))
            if obstacle_dist(p, poles) < 0.5 or math.dist(p, SPAWN) < 0.8 or any(math.dist(p, q) < 1.5 for q in poles):
                continue
            if any(obstacle_dist((p[0] + s * n[0], p[1] + s * n[1]), poles) < 0.45 for s in (0.45, 0.75, 1.0)):
                continue
            if any(math.dist(p, (t[1] + s * math.cos(t[3]), t[2] + s * math.sin(t[3]))) < 0.5 for t in tags for s in (0.45, 1.0)):
                continue
            poles.append(p); tags.append((tag_id, p[0], p[1], yaw)); break
        else:
            ok = False; break
    if ok:
        break

w = open(SRC).read()
w = w.replace("<!-- Gazon synthetique : visuel seulement, le contact reste celui du sol -->",
              "<!-- Sol dur clair (parquet / carrelage) : visuel seulement -->")
w = w.replace("<model name='grass'>", "<model name='sol'>")
w = w.replace("<ambient>0.2 0.5 0.2 1</ambient>\n            <diffuse>0.2 0.5 0.2 1</diffuse>",
              "<ambient>0.80 0.70 0.58 1</ambient>\n            <diffuse>0.80 0.70 0.58 1</diffuse>")
debut = w.index("    <!-- Disposition aleatoire")
fin = w.index("  </world>")
inc = lambda uri, nom, x, y, z, yaw: (f"    <include>\n      <uri>model://{uri}</uri>\n      <name>{nom}</name>\n"
                                      f"      <pose>{x:.2f} {y:.2f} {z} 0 0 {yaw:.3f}</pose>\n    </include>\n")
corps = (f"    <!-- Arene finale (photos du 4 nov) : 2 tables le long des murets, 3 chaises en diagonale.\n"
         f"         Positions estimees sur les photos. Tags tires au hasard (graine {seed}) : poteau degage,\n"
         f"         zone de 0.45 a 1 m devant chaque tag libre, 1.5 m entre tags, depart (-1.2, -1.9) degage. -->\n\n")
corps += "".join(inc("school_desk", f"desk_{i}", c[0], c[1], 0, y) for i, (c, y) in enumerate(desks, 1))
corps += "".join(inc("plastic_chair", f"chair_{i}", c[0], c[1], 0, y) for i, (c, y) in enumerate(chairs, 1))
corps += "".join(inc(f"apriltag_{t}", f"apriltag_{t}", x, y, 1.0, a) for t, x, y, a in tags)
w = w[:debut] + corps + w[fin:]
open(DST, 'w').write(w)
print("graine", seed)
for t in tags: print(f"tag {t[0]} {t[1]:.2f} {t[2]:.2f} {t[3]:.3f}")
