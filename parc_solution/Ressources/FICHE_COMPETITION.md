# Fiche compétition MA64 (branche peace/finale)

Ouvrir les terminaux : `~/Projects/parc_ingé_league/src/parc_solution/outils_robot/ouvrir_terminaux.sh`
Avant tout : vérifier l'adresse du robot dans `outils_robot/robot.conf` (`export ROBOT=sitoe@IP`).

## Déroulé (volets, dans l'ordre)

| # | Volet | Ce qu'il doit afficher | Si ce n'est pas le cas |
|---|---|---|---|
| 1 | **deploy** | `nav_solution compile`, `couche DDS : rmw_cyclonedds_cpp`, `tout est present` | Mauvaise IP : corriger `robot.conf`. Erreur de compilation : envoyer la sortie |
| 2 | **rmw** | `OK : rmw_cyclonedds_cpp recoit /scan` | `AUCUN DDS` : `echo $ROS_DOMAIN_ID` sur le robot, vérifier le Wi-Fi |
| 3 | **bag** | `Topics discovery stopped` | Pas bloquant, on continue |
| 4 | **carto**, puis **camera** | `OK : camera2 publie ... le detecteur recoit (X Hz)` | `PROBLEME camera_info` : relancer **carto**, puis **camera** |
| 5 | (RViz, pendant **carto**) | Un objet posé **à gauche** du robot apparaît **à gauche** | Il apparaît à droite : prévenir tout de suite (LiDAR) |
| 6 | Ctrl+C dans **carto** | | |
| 7 | Poser le robot | **Coin sous le panier, tourné vers le gazon** (même placement à chaque essai) | |
| 8 | **course** | `indices : .../finale_indices.yaml`, puis `attente du depart` | `Failed to find a free participant index` : refaire **deploy** |
| 9 | **etat** | `state: 1` (le juge a armé) | |
| 10 | **depart** | Dans **course** : `DEPART publie` | Rien : relancer **depart** |
| 11 | **sauver** (pendant ou après la course) | Carte enregistrée | Taper la commande **sans** le `$ ` au début |
| 12 | **masque** | Dessiner les 3 plateaux de table, puis `s` | Pour l'essai suivant seulement |
| 13 | **rapatrier** | Copie des journaux, cartes et bags | |

## Pendant la course
- Ouverture : environ 2,7 m tout droit vers le gazon, virage à gauche, puis tour de 360°.
- Le journal **course** affiche `ouverture finie, tags mesures : [...]`, puis `VISITE signalee : tag N`.
- Risque principal : contact avec une table autour du tag 4. Au 2e essai, avec le masque dessiné, ce risque baisse.

## Si ça bloque
- Le robot ne bouge plus depuis plus d'une minute : Ctrl+C dans **course**, puis relancer **course** et **depart**.
- Pas de départ vocal : le robot n'a pas de micro (pas de `/audio`), le départ se fait avec **depart**.
- Ne jamais relancer **nettoyer** pendant la compétition.
