"""phase2_hard.launch.py du simulateur officiel, avec moins d'obstacles (proche de l'arene de competition).

Charge le launch officiel tel quel et ne change que le nombre d'obstacles passe au generateur :
    ros2 launch .../phase2_simple.launch.py seed:=5 gui:=false rviz:=false audio:=false
Quantites (variables d'environnement, defaut : 2 tables avec chaises, 0 meuble mural, 2 objets libres) :
    MA64_TABLES, MA64_MURAUX, MA64_LIBRES
Memes sorties que l'officiel : /tmp/parc_worlds/parc_random_seed<N>.sdf et .json
"""
import functools
import importlib.util
import os

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    officiel = os.path.join(FindPackageShare('parc_robot_bringup').find('parc_robot_bringup'),
                            'launch', 'phase2_hard.launch.py')
    spec = importlib.util.spec_from_file_location('phase2_hard_officiel', officiel)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.generate = functools.partial(
        module.generate,
        obstacle_sets=int(os.environ.get('MA64_TABLES', 2)),
        wall_items=int(os.environ.get('MA64_MURAUX', 0)),
        free_items=int(os.environ.get('MA64_LIBRES', 2)))
    return module.generate_launch_description()
