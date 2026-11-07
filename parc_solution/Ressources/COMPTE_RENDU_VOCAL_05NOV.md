# Compte rendu : départ vocal et vérification de la branche de Peace (5 novembre)

Branche `sun/voice_start` = `peace/phase-reelle` + simulateur officiel + les correctifs ci-dessous.

## But
- Gagner les **+5 points du départ vocal** sans risquer de **faux départ** : la course serait recommencée et le bonus perdu.
- Vérifier, en le faisant tourner, ce que Peace a déjà fait, pour compléter son travail au lieu de le refaire.

## Ce qui a été vérifié sur sa branche
Course complète de son code dans le simulateur officiel (seed 7), mesurée par son évaluateur à partir de la vraie pose Gazebo :

| Mesure | Résultat |
|---|---|
| Arbitre | FINISHED, 4/4 tags (ordre 4, 7, 9, 1), pas de faux départ |
| Contacts | aucun ; inclinaison max 2,5° (celle du robot au repos) |
| Distance au tag | avant du robot à 0,48 m, centre à 0,68 m |
| Angle face au tag | 1,6 à 1,8° (tolérance ±12°) |
| Durée | 3 min 45 de temps simulé |

Condition du test : odométrie étalonnée comme chez lui (`wheel_separation` 0,445, dans une copie locale seulement).

## Méthode pour le départ vocal
1. Enregistrements au micro du laptop, 16 kHz mono : « allez » seul, « go » seul, 60 s de parole sans le mot (avec des mots proches : alors, allons, bon, cool…), bruit.
2. Chaque règle passée sur les 4 fichiers avec Vosk. On compte les détections et les faux départs.
3. La meilleure règle mise dans son nœud, puis testée à travers le vrai nœud ROS, puis de bout en bout dans le simulateur officiel : micro simulé → nœud vocal → mission → arbitre.

## Ce qui n'a pas marché, et comment c'est réglé
| Problème | Mesure | Réglage |
|---|---|---|
| Premiers enregistrements inutilisables | 32 % des échantillons saturés : gain du micro au maximum (+30 dB et +30 dB) | gain baissé (boost 0, capture 30 %), enregistrements refaits |
| Règle d'origine : grammaire go/allez/vai, mot accepté n'importe où dans la phrase | **10 faux départs en 60 s** de parole sans le mot (« nous allons… », « alors »…) | vocabulaire libre, mot **« allez » seul**, la phrase entière doit être le mot |
| « go » mal reconnu | vocabulaire libre : 0 « go » reconnu sur environ 10 | mot retenu : **« allez »** |
| Grammaire limitée à « allez » | 3 faux départs en 60 s | abandonnée |
| La mission gardait un mot entendu **avant l'armement** : le robot serait parti dès l'armement | lecture de `on_start_word` | mot ignoré tant que la course n'est pas armée (log `mot de depart ignore`) |
| Vosk absent du robot, sans internet sur place | — | paquets aarch64 et modèle français prêts dans `~/parc_offline` ; l'installation dans un dossier d'équipe est testée hors ligne sur le laptop. À intégrer avec Peace |

## Résultats
| Test | Résultat |
|---|---|
| Nœud ROS, `allez.wav` | 14 départs pour environ 16 « allez » dits |
| Nœud ROS, parole sans le mot, `go.wav`, bruit | **0 faux départ** |
| Bout en bout, simulateur officiel | « allez » avant l'armement : ignoré, le robot ne part pas. Après l'armement : départ, arbitre `start_method: 1` (voix), `false_start: false` |
| Paquet envoyé au robot (`preparer_envoi.sh`), avec les deux fichiers modifiés | 4 vérifications OK |

## Propositions à montrer à Peace (rien n'est fusionné dans sa branche)
Son guide, `deploy.sh`, `lancer.sh` et `robot.launch.py` restent exactement les siens. Sur `sun/voice_start`, seuls deux fichiers diffèrent des siens :
- `nav_solution/demarrage_vocal.py` : la règle « allez » seul, en vocabulaire libre ;
- `nav_solution/mission_node.py` : un mot entendu avant l'armement est ignoré.

L'installation de Vosk sur le robot (paquets aarch64 et modèle français, dans `~/parc_offline`) et le test du micro sont à décider avec lui.

## Reste à faire
- **Au créneau :** écouter le micro du robot, qui ne l'a encore jamais été. Dire « allez » seul 5 fois (5 départs attendus), puis parler 20 s sans le mot (0 attendu).
- **À confirmer par écrit avec Evans :** les 0,5 m se mesurent-ils depuis le centre du robot (texte du brief) ou depuis son avant (règle orale du 5/11, que suit la mission) ?
