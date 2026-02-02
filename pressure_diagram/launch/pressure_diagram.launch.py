#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pressure_diagram',
            executable='pressure_diagram',
            name='pressure_diagram_node',
            output='screen',
            parameters=[{

            }],
        ),

    ])