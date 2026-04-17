from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mocap_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pydantic>=2.0", "pyyaml"],
    zip_safe=True,
    maintainer="Timothee Carecchio",
    maintainer_email="timothee.Carecchio@inria.fr",
    description="ROS 2 bridge for Qualisys Motion Capture – multi-body, YAML-driven.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "mocap_node = mocap_ros.mocap_node:main",
        ],
    },
)
