from __future__ import annotations

import os
from glob import glob

from setuptools import setup

package_name = 'agv_remote'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.perspective')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakub Czech',
    maintainer_email='czechjakub@icloud.com',
    description='Package for visualizing and controlling AGV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
