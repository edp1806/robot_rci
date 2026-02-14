#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nœud MGD
        Node(
            package='robot_rci_control',
            executable='mgd_node',
            name='mgd_node',
            output='screen'
        ),

        # Nœud MGI
        Node(
            package='robot_rci_control',
            executable='mgi_node',
            name='mgi_node',
            output='screen'
        ),

        # Nœud Workspace Publisher
        Node(
            package='robot_rci_control',
            executable='workspace_publisher',
            name='workspace_publisher',
            output='screen'
        ),
    ])
