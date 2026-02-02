#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数：是否显示压力图
    declare_show_diagram = DeclareLaunchArgument(
        'show_pressure_diagram',
        default_value='true',
        description='是否启动压力图窗口: true | false'
    )

    # 根据 show_pressure_diagram 参数决定是否包含 pressure_diagram launch 文件
    pressure_diagram_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pressure_diagram'),
                'launch',
                'pressure_diagram.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('show_pressure_diagram'))
    )

    return LaunchDescription([
        declare_show_diagram,
        pressure_diagram_launch,
        Node(
            package='gui_control',
            executable='gui_control',
            name='left_hand_control_node',
            output='screen',
            parameters=[{
                'hand_type': 'right',  # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "L6",  # O6\L6\L7\L10\L20\G20(工业版)\L21 字母为大写
                'topic_hz': 30, # topic发布频率
                'is_touch': True, # 是否有压力传感器
                'is_arc': True, # 是否发布弧度值topic
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
