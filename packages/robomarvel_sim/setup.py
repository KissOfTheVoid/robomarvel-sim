import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robomarvel_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "worlds"), glob("config/worlds/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ivan Kopylov",
    maintainer_email="walker096@gmail.com",
    description="Симуляция Robomarvel",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_hwnode = robomarvel_sim.sim_hwnode:main",
            "sim_lidar = robomarvel_sim.sim_lidar:main",
            "sim_detection = robomarvel_sim.sim_detection:main",
            "goal_proxy = robomarvel_sim.goal_proxy:main",
        ],
    },
)
