#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 假设你有一个名为 "example_node" 的节点，它位于 linker_hand_ros2_sdk 包中
        # 并且该节点的可执行文件已经编译好，通常位于 src 目录下（这里需要确认实际位置）
        Node(
            package='matrix_touch_gui',
            executable='matrix_touch_gui',
            name='matrix_touch_gui',
            output='screen',
            parameters=[{
                'hand_type': 'right',
                'hand_joint': "L6",  # O6\L6\L7\L10\L20\L21
                'topic_hz': 30,
                'is_touch': True,
            }],
        ),
    ])
