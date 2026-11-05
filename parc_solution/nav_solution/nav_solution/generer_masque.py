#! /usr/bin/env python3

##### Masque de zones interdites (filtre keepout de Nav2), comme en phase 1.
#    Le LiDAR ne voit que les pieds des tables et des chaises : leurs plateaux et
#    leurs assises debordent. On dessine ces surfaces sur la carte enregistree ;
#    Nav2 les traite alors comme des obstacles (keepout_filter, avant l'inflation).
#
#    A la souris, sur la carte faite pendant les essais :
#      ros2 run nav_solution generer_masque ~/MA64/cartes/arene.yaml
#        clic gauche : ajoute un coin   | clic droit ou Entree : ferme le polygone
#        u : annule le dernier coin     | s : enregistre et quitte | q : quitte sans enregistrer
#    Depuis une liste de polygones (simulation, tests) :
#      ros2 run nav_solution generer_masque carte.yaml --polygones obstacles.yaml
#        obstacles.yaml : {polygones: [[[x, y], [x, y], ...], ...]}  (repere map, en m)
#    Ecrit <carte>_keepout.pgm et <carte>_keepout.yaml (meme grille que la carte).
#    Le masque est dans le repere map : la course doit partir du meme point que la
#    cartographie (scotch au sol), comme pour les positions des tags.

import argparse
import os

import numpy as np
import yaml

try:
    import cv2
except ImportError:
    cv2 = None


def lire_carte(chemin):
    with open(chemin) as f:
        info = yaml.safe_load(f)
    image = info['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(chemin)), image)
    if cv2 is None:
        raise SystemExit('OpenCV (python3-opencv) est necessaire')
    img = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f'image de carte illisible : {image}')
    return info, img


def vers_pixels(poly, info, hauteur):
    """Coordonnees map (m) -> pixels de l'image (ligne 0 en haut)."""
    res, ox, oy = info['resolution'], info['origin'][0], info['origin'][1]
    return np.array([[(x - ox) / res, hauteur - (y - oy) / res] for x, y in poly], dtype=np.float32)


def vers_metres(px, info, hauteur):
    res, ox, oy = info['resolution'], info['origin'][0], info['origin'][1]
    return [round(ox + px[0] * res, 3), round(oy + (hauteur - px[1]) * res, 3)]


def ecrire_masque(polygones, info, forme, marge, sortie):
    h, w = forme
    masque = np.full((h, w), 254, np.uint8)
    for poly in polygones:
        pts = np.round(vers_pixels(poly, info, h)).astype(np.int32)
        cv2.fillPoly(masque, [pts], 0)
    if marge > 0:
        r = int(np.ceil(marge / info['resolution']))
        noyau = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        masque = cv2.erode(masque, noyau)   # erosion du blanc = dilatation des obstacles noirs
    cv2.imwrite(sortie + '.pgm', masque)
    with open(sortie + '.yaml', 'w') as f:
        f.write('# Masque keepout (generer_masque) : noir = zone interdite a Nav2\n')
        f.write(f'# {len(polygones)} polygone(s), marge {marge} m\n')
        yaml.safe_dump({'image': os.path.basename(sortie) + '.pgm', 'mode': 'trinary',
                        'resolution': info['resolution'], 'origin': info['origin'], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.196}, f, sort_keys=False)
    with open(sortie + '_polygones.yaml', 'w') as f:
        yaml.safe_dump({'polygones': polygones}, f)
    print(f'masque ecrit : {sortie}.yaml ({len(polygones)} polygone(s), marge {marge} m)')


def dessiner(info, img):
    """Fenetre de dessin : renvoie la liste des polygones (repere map) ou None."""
    zoom = max(1, int(800 / max(img.shape)))
    polygones, courant = [], []
    fond = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def souris(evt, x, y, flags, _):
        if evt == cv2.EVENT_LBUTTONDOWN:
            courant.append(vers_metres((x / zoom, y / zoom), info, img.shape[0]))
        elif evt == cv2.EVENT_RBUTTONDOWN and len(courant) >= 3:
            polygones.append(list(courant))
            courant.clear()

    nom = 'Masque keepout - clic gauche : coin, clic droit / Entree : fermer, u : annuler, s : enregistrer'
    cv2.namedWindow(nom)
    cv2.setMouseCallback(nom, souris)
    while True:
        vue = cv2.resize(fond, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_NEAREST)
        for poly in polygones:
            pts = np.round(vers_pixels(poly, info, img.shape[0]) * zoom).astype(np.int32)
            cv2.fillPoly(vue, [pts], (0, 0, 220))
        if courant:
            pts = np.round(vers_pixels(courant, info, img.shape[0]) * zoom).astype(np.int32)
            cv2.polylines(vue, [pts], False, (0, 160, 255), 2)
            for p in pts:
                cv2.circle(vue, tuple(int(v) for v in p), 3, (0, 160, 255), -1)
        cv2.imshow(nom, vue)
        k = cv2.waitKey(30) & 0xFF
        if k in (13, 10) and len(courant) >= 3:
            polygones.append(list(courant))
            courant.clear()
        elif k == ord('u'):
            if courant:
                courant.pop()
            elif polygones:
                polygones.pop()
        elif k == ord('s'):
            cv2.destroyAllWindows()
            return polygones
        elif k == ord('q') or cv2.getWindowProperty(nom, cv2.WND_PROP_VISIBLE) < 1:
            cv2.destroyAllWindows()
            return None


def main():
    ap = argparse.ArgumentParser(description='Masque keepout Nav2 dessine sur une carte enregistree')
    ap.add_argument('carte', help='YAML de la carte (map_saver_cli)')
    ap.add_argument('--polygones', help='YAML {polygones: [[[x, y], ...], ...]} : pas de fenetre')
    ap.add_argument('--marge', type=float, default=0.05, help='marge ajoutee autour de chaque polygone (m)')
    ap.add_argument('--sortie', help='prefixe des fichiers (defaut : <carte>_keepout)')
    a = ap.parse_args()
    info, img = lire_carte(a.carte)
    sortie = a.sortie or os.path.splitext(os.path.abspath(a.carte))[0] + '_keepout'
    if a.polygones:
        with open(a.polygones) as f:
            polygones = yaml.safe_load(f)['polygones']
    else:
        polygones = dessiner(info, img)
        if polygones is None:
            print('abandon : rien n\'est enregistre')
            return
    ecrire_masque(polygones, info, img.shape, a.marge, sortie)


if __name__ == '__main__':
    main()
