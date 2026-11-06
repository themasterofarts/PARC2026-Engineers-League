from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'maps_zone_interdi'), glob('maps_zone_interdi/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='djo9800',
    maintainer_email='siradjosy1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_solution = nav_solution.task_solution:main',
            'apriltag_tf_node = nav_solution.apriltag_tf_node:main',
            'mission_node = nav_solution.mission_node:main',
            'faux_arbitre = nav_solution.faux_arbitre:main',
            'sauver_tags = nav_solution.sauver_tags:main',
            'generer_masque = nav_solution.generer_masque:main',
            'demarrage_vocal = nav_solution.demarrage_vocal:main',
            'scan_retourne = nav_solution.scan_retourne:main',
        ],
    },
)
