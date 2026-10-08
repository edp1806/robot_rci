#!/usr/bin/env python3
"""
Lancement du robot RCI avec visualisation Foxglove (au lieu de RViz).

Démarre :
  - robot_state_publisher (URDF -> /robot_description, /tf, /tf_static)
  - foxglove_bridge (WebSocket sur ws://localhost:8765)
  - le panneau de contrôle Tkinter (désactivable avec gui:=false)

Utilisation :
  ros2 launch robot_rci_description foxglove.launch.py
  ros2 launch robot_rci_description foxglove.launch.py gui:=false port:=8765
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_rci_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_rci.urdf.xacro')

    gui = LaunchConfiguration('gui')
    port = LaunchConfiguration('port')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': port,
            'address': '127.0.0.1',  # local uniquement
        }]
    )

    gui_node = Node(
        package='robot_rci_gui',
        executable='control_panel',
        name='robot_control_gui',
        output='screen',
        condition=IfCondition(gui)
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Lancer le panneau de contrôle Tkinter'),
        DeclareLaunchArgument('port', default_value='8765',
                              description='Port WebSocket de foxglove_bridge'),
        robot_state_publisher_node,
        foxglove_bridge_node,
        gui_node,
    ])
