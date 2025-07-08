#!/usr/bin/env python3
import os
import sys
from setuptools import find_packages, setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.2.13',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/detection_models', ['gui/detection_models/yolo_multi_seg_n.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='za',
    maintainer_email='529768926@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gui = gui.main:main',
        ],
    },
)
