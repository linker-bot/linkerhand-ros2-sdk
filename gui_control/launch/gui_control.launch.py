#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_control',
            executable='gui_control',
            name='left_hand_control_node',
            output='screen',
            parameters=[{
                'hand_type': 'right',  # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "L10",  # O6\L6\L7\L10\L20\L21 字母为大写
                'topic_hz': 30, # topic发布频率
                'is_touch': True, # 是否有压力传感器
                'is_arc': False, # 是否发布弧度值topic
            }],
        ),
        # Node(
        #     package='gui_control',
        #     executable='gui_control',
        #     name='right_hand_control_node',
        #     output='screen',
        #     parameters=[{
        #         'hand_type': 'right',
        #         'hand_joint': "L10",
        #         'topic_hz': 30,
        #         'is_touch': True,
        #     }],
        # ),
    ])
