#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 假设你有一个名为 "example_node" 的节点，它位于 linker_hand_ros2_sdk 包中
        # 并且该节点的可执行文件已经编译好，通常位于 src 目录下（这里需要确认实际位置）
        Node(
            package='gui_control',
            executable='gui_control',
            name='left_hand_control_node',
            output='screen',
            parameters=[{
                'hand_type': 'left',
                'hand_joint': "L20",
                'topic_hz': 30,
                'is_touch': True,
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
