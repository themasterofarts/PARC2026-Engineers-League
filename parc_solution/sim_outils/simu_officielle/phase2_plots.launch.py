"""phase2.launch.py du simulateur officiel, avec notre replique de l'arene reelle (photo du 6 nov).

Arene fixe officielle sans tables ni chaises, plus 3 plots (un grand, deux petits) :
arene_reelle_plots.sdf. Le reste du lancement officiel est inchange.
"""
import os

from launch_ros.substitutions import FindPackageShare

# MA64_MONDE : autre monde (ex. arene_finale.sdf), sinon la replique aux plots
MONDE = os.environ.get('MA64_MONDE') or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'arene_reelle_plots.sdf')


def generate_launch_description():
    officiel = os.path.join(FindPackageShare('parc_robot_bringup').find('parc_robot_bringup'),
                            'launch', 'phase2.launch.py')
    source = open(officiel).read()
    ancien = 'os.path.join(pkg_path, "worlds/nba_sim.sdf")'
    assert ancien in source, 'phase2.launch.py a change : chemin du monde introuvable'
    espace = {'__file__': officiel}
    source = source.replace(ancien, repr(MONDE))
    if os.environ.get('MA64_GUI', '1') != '1':
        # sans fenetres (comme phase2_hard gui:=false) : Gazebo serveur seul, sans la RViz officielle
        for a, b in (('"-r -v4 "', '"-r -s --headless-rendering -v2 "'),
                     ('ld.add_action(start_rviz_cmd)', 'pass'),
                     ('ld.add_action(delayed_camera_adjustment)', 'pass')):
            assert a in source, f'phase2.launch.py a change : {a} introuvable'
            source = source.replace(a, b)
    exec(compile(source, officiel, 'exec'), espace)
    return espace['generate_launch_description']()
