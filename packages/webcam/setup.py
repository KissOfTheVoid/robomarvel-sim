from setuptools import find_packages, setup
from glob import glob
import os

package_name = "webcam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ivan Kopylov",
    maintainer_email="walker096@gmail.com",
    description="Видеопоток RTSP в топик ROS 2",
    license="MIT",
    extras_require={},
    entry_points={
        "console_scripts": ["rtsp2ros = webcam.rtsp2ros:main"],
    },
)
