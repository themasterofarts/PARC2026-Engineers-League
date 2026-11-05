##### Couverture camera : ce que la camera a deja regarde, pour chercher un tag la ou elle n'a pas vu.
#    Les tags sont colles sur des surfaces (murs, poteaux) : ce sont les cases occupees de la carte
#    SLAM, au bord de la zone ou le robot peut aller. Une case est "vue" quand elle a ete dans le
#    champ de la camera, a moins de portee, sans obstacle entre les deux, et vue sous moins de
#    angle_max de sa normale : les conditions dans lesquelles un tag y serait detecte. Un poteau
#    n'est donc couvert qu'une fois vu de plusieurs cotes (chaque face a sa normale).
#    Le prochain point de vue est celui qui voit le plus de cases encore non vues, moins la route.
#    Repere : celui de la carte (map). Pas de ROS ici : numpy et OpenCV seulement.

import math

import cv2
import numpy as np


class Couverture:

    def __init__(self, portee=4.0, angle_max_deg=55.0, hfov_deg=55.0):
        self.portee = portee
        self.cos_max = math.cos(math.radians(angle_max_deg))
        self.demi_fov = math.radians(hfov_deg) / 2
        self.vues = set()      # cases deja vues, cles (ix, iy) en cases entieres du repere map
        self.res = None
        self.cases = None      # (N, 2) centres des surfaces en metres
        self.cles = []         # cle de chaque surface
        self.normales = None   # (N, 2)
        self.occ = None        # carte booleenne des obstacles (ligne de vue)
        self.ox = self.oy = 0.0

    def maj_carte(self, data, res, ox, oy, accessible):
        """data : carte SLAM (h, w) en int (-1 inconnu, 0..100) ; accessible(x, y) -> tableau bool
        des points ou le robot peut aller (zone atteignable de la costmap). Les surfaces retenues
        sont les cases occupees a moins de 0.35 m d'une case accessible."""
        occ = data >= 65
        if not occ.any():
            self.cases = None
            return
        flou = cv2.GaussianBlur(occ.astype(np.float32), (5, 5), 1.0)
        gx = cv2.Sobel(flou, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(flou, cv2.CV_32F, 0, 1, ksize=3)
        jj, ii = np.nonzero(occ)
        x = ox + (ii + 0.5) * res
        y = oy + (jj + 0.5) * res
        # pres d'une zone accessible : on teste 8 voisins a 0.35 m (un muret, un poteau, un meuble)
        pres = np.zeros(len(x), bool)
        for a in np.arange(8) * math.pi / 4:
            pres |= accessible(x + 0.35 * math.cos(a), y + 0.35 * math.sin(a))
        n = np.column_stack([gx[jj, ii], gy[jj, ii]])
        norme = np.linalg.norm(n, axis=1, keepdims=True)
        n = np.where(norme > 1e-3, n / np.maximum(norme, 1e-6), 0.0)   # normale nulle : toute incidence
        self.res, self.ox, self.oy, self.occ = res, ox, oy, occ
        self.cases = np.column_stack([x, y])[pres]
        self.normales = n[pres]
        self.cles = [(int(round(cx / res)), int(round(cy / res))) for cx, cy in self.cases]

    def visibles(self, px, py):
        """Masque des surfaces vues correctement depuis (px, py), toutes directions confondues."""
        if self.cases is None:
            return None
        d = self.cases - np.array([px, py])
        dist = np.linalg.norm(d, axis=1)
        ok = (dist < self.portee) & (dist > 0.2)
        v = -d / np.maximum(dist[:, None], 1e-6)                     # case -> camera
        sans_normale = np.abs(self.normales).sum(axis=1) == 0
        ok &= sans_normale | (np.abs(np.sum(self.normales * v, axis=1)) >= self.cos_max)
        idx = np.nonzero(ok)[0]
        if idx.size:
            ok[idx] = self.ligne_de_vue(px, py, self.cases[idx], dist[idx])
        return ok

    def ligne_de_vue(self, px, py, cibles, dist):
        """True si aucun obstacle de la carte entre (px, py) et chaque cible (hors ses 2 dernieres
        cases, qui font partie de la surface elle-meme)."""
        pas = self.res / 2
        n = int(np.ceil(dist.max() / pas)) + 1
        t = np.linspace(0.0, 1.0, n)[None, :]
        xs = px + (cibles[:, 0:1] - px) * t
        ys = py + (cibles[:, 1:2] - py) * t
        i = ((xs - self.ox) / self.res).astype(int)
        j = ((ys - self.oy) / self.res).astype(int)
        h, w = self.occ.shape
        dedans = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        bloque = np.zeros_like(dedans)
        bloque[dedans] = self.occ[j[dedans], i[dedans]]
        # on ignore le debut (le robot) et la fin du segment (la surface visee)
        utile = (t * dist[:, None] > 0.25) & (t * dist[:, None] < dist[:, None] - 2 * self.res)
        return ~np.any(bloque & utile, axis=1)

    def observer(self, px, py, cap):
        """Marque les surfaces dans le champ de la camera placee en (px, py), cap donne."""
        ok = self.visibles(px, py)
        if ok is None:
            return
        d = self.cases - np.array([px, py])
        a = np.arctan2(d[:, 1], d[:, 0]) - cap
        ok &= np.abs(np.arctan2(np.sin(a), np.cos(a))) <= self.demi_fov
        for k in np.nonzero(ok)[0]:
            self.vues.add(self.cles[k])

    def non_vues(self):
        if self.cases is None:
            return np.zeros(0, bool)
        return np.array([c not in self.vues for c in self.cles], bool)

    def meilleur_point(self, candidats, robot, poids_route=6.0):
        """Parmi les points candidats (M, 2), celui qui voit le plus de surfaces non vues dans un
        meme champ de camera, moins poids_route cases par metre de route. Renvoie (x, y, cap, gain)
        ou None."""
        if self.cases is None or len(candidats) == 0:
            return None
        reste = self.non_vues()
        if not reste.any():
            return None
        meilleur = None
        for x, y in candidats:
            ok = self.visibles(x, y)
            ok &= reste
            if not ok.any():
                continue
            d = self.cases[ok] - np.array([x, y])
            angles = np.sort(np.arctan2(d[:, 1], d[:, 0]))
            # meilleure fenetre de largeur fov sur le cercle des directions
            double = np.concatenate([angles, angles + 2 * math.pi])
            fin = np.searchsorted(double, angles + 2 * self.demi_fov, side='right')
            k = int(np.argmax(fin - np.arange(len(angles))))
            gain = int(fin[k] - k)
            cap = angles[k] + self.demi_fov
            score = gain - poids_route * math.hypot(x - robot[0], y - robot[1])
            if meilleur is None or score > meilleur[4]:
                meilleur = (float(x), float(y), math.atan2(math.sin(cap), math.cos(cap)), gain, score)
        return None if meilleur is None else meilleur[:4]

    def remise_a_zero(self):
        self.vues.clear()
