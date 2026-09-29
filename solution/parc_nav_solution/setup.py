import os
from glob import glob

from setuptools import find_packages, setup

package_name = "parc_nav_solution"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your-team",
    maintainer_email="you@example.com",
    description="PARC 2026 Engineers League navigation task solution",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_solution.py = parc_nav_solution.task_solution:main",
            "imu_odom_corrector = parc_nav_solution.imu_odom_corrector:main",
            "depth_obstacles = parc_nav_solution.depth_obstacles:main",
        ],
    },
)
