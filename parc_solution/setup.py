from glob import glob
from setuptools import find_packages, setup


package_name = "parc_solution"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PARC Team",
    maintainer_email="team@example.com",
    description="Autonomous navigation solution for the PARC 2026 simulation task.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task_solution = parc_solution.task_solution:main",
            "task_solution.py = parc_solution.task_solution:main",
            "cmd_vel_relay = parc_solution.cmd_vel_relay:main",
            "self_return_filter = parc_solution.self_return_filter:main",
        ],
    },
)
