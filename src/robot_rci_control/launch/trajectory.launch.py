#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nœud de trajectoire circulaire
        Node(
            package='robot_rci_control',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            parameters=[{
                'center_x': -1.70,
                'center_y': -1.10,
                'center_z': 0.55,
                'radius': 0.20,
                'num_points': 60,
                'frequency': 10.0
            }]
        ),

        # Nœud MGI pour convertir les positions en commandes articulaires
        Node(
            package='robot_rci_control',
            executable='mgi_node',
            name='mgi_node',
            output='screen'
        ),
    ])
