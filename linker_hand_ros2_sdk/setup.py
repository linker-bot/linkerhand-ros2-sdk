#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'linker_hand_ros2_sdk'

this_dir = os.path.abspath(os.path.dirname(__file__))
custom_dir = os.path.join(this_dir, package_name, "LinkerHand")

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

# for root, dirs, files in os.walk(custom_dir):
#     if files:
#         relative_path = os.path.relpath(root, os.path.join(this_dir, package_name))
#         target_path = os.path.join('share', package_name, relative_path)
#         # 修复这里：路径必须是相对路径
#         files_full_path = [os.path.relpath(os.path.join(root, f), start=os.getcwd()) for f in files]
#         data_files.append((target_path, files_full_path))
        

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linker-robot',
    maintainer_email='linker-robot@todo.todo',
    description='ROS2 SDK for Linker Hand',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'linker_hand_sdk = linker_hand_ros2_sdk.linker_hand:main',
            'linker_hand_advanced_l6 = linker_hand_ros2_sdk.linker_hand_advanced_l6:main',
            'linker_hand_advanced_l7 = linker_hand_ros2_sdk.linker_hand_advanced_l7:main',
            'linker_hand_advanced_o6 = linker_hand_ros2_sdk.linker_hand_advanced_o6:main',
            'linker_hand_advanced_g20 = linker_hand_ros2_sdk.linker_hand_advanced_g20:main',
        ],
    },
)
