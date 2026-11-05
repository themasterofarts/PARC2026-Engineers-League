#!/bin/bash
# Ouvre les trois fenetres Terminator du creneau (dans l ordre de lancement), une commande pre-tapee par volet (Entree pour lancer).
# Remplir d'abord robot.conf (ROBOT=utilisateur@ip, CAMERA, TAILLE).
ICI="$(dirname "$(readlink -f "$0")")"
# environnement propre (le terminal de VS Code, installe en snap, fait planter les applications graphiques)
[ -n "$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH \
      GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR SNAP SNAP_LIBRARY_PATH
for l in ma64_1_preparation ma64_2_cartographie ma64_3_course; do
  setsid terminator -u -g "$ICI/terminator_ma64.conf" -l "$l" > /dev/null 2>&1 &
done
