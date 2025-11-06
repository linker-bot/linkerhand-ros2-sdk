#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_ros2_sdk',
            executable='linker_hand_sdk',
            name='linker_hand_sdk_left',
            output='screen',
            parameters=[{
                'hand_type': 'left', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "G20", # O6\L6P\L6\L7\L10\L20\G20(工业版)\L21 字母为大写
                'is_touch': True, # 配置Linker Hand灵巧手是否有压力传感器 True | False
                'can': 'can0', # 这里需要修改为实际的CAN总线名称 如果是win系统则类似于 PCAN_USBBUS1
                "modbus": "None" # "None" | "/dev/ttyUSB0" 这里需要修改为实际的Modbus总线名称 如果是win系统则 COM* Ubuntu则为/dev/ttyUSB*
            }],
        ),

        Node(
            package='linker_hand_ros2_sdk',
            executable='linker_hand_sdk',
            name='linker_hand_sdk_right',
            output='screen',
            parameters=[{
                'hand_type': 'right', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "G20", # O6\L6P\L6\L7\L10\L20\G20(工业版)\L21 字母为大写
                'is_touch': True, # 配置Linker Hand灵巧手是否有压力传感器 True | False
                'can': 'can1', # 这里需要修改为实际的CAN总线名称 如果是win系统则类似于 PCAN_USBBUS1
                "modbus": "None" # "None" | "/dev/ttyUSB0" 这里需要修改为实际的Modbus总线名称 如果是win系统则 COM* Ubuntu则为/dev/ttyUSB*
            }],
        ),
    ])
