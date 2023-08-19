from __future__ import annotations

import os
from glob import glob

from setuptools import setup

package_name = "agv_waypoint_sender"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jakub Czech",
    maintainer_email="czechjakub@icloud.com",
    description="Waypoints sender fo AGV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_sender = agv_waypoint_sender.sender:main",
            "analyzer = agv_waypoint_sender.analyzer:main",
        ],
    },
)
