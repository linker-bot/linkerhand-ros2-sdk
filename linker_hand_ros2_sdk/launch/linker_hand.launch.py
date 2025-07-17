#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_ros2_sdk',
            executable='linker_hand_sdk',
            name='linker_hand_sdk',
            output='screen',
            parameters=[{
                'hand_type': 'left',
                'hand_joint': "L10",
                'is_touch': True,
                'can': 'can0', # 这里需要修改为实际的CAN总线名称 如果是win系统则 PCAN_USBBUS1
            }],
        ),
    ])
