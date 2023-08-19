from setuptools import setup
import os
from glob import glob

package_name = "kinco_driver"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jakub Czech",
    maintainer_email="czechjakub@icloud.com",
    description="ROS2 Humble package for controll Kinco DC Servo FD124S",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["kinco_driver_node = kinco_driver.kinco_driver_node:main"],
    },
)
