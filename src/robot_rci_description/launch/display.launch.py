#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Chemins
    pkg_share = get_package_share_directory('robot_rci_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_rci.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    # Arguments de lancement
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Charger le URDF avec xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Nœud robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Nœud RViz2
    # Sous GNOME Wayland avec mise à l'échelle fractionnaire, la vue 3D de RViz2
    # clignote (une image sur deux noire). Forcer X11 (XWayland) et désactiver
    # le scaling HiDPI de Qt supprime le clignotement. Désactivable avec
    # qt_scaling_fix:=false (par ex. sur un écran HiDPI en session X11).
    qt_scaling_fix = LaunchConfiguration('qt_scaling_fix')
    qt_fix_env = {
        'QT_QPA_PLATFORM': 'xcb',
        'QT_ENABLE_HIGHDPI_SCALING': '0',
        'QT_SCALE_FACTOR': '1',
        'QT_SCREEN_SCALE_FACTORS': '1',
    }

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        additional_env=qt_fix_env,
        condition=IfCondition(qt_scaling_fix)
    )

    rviz_node_no_fix = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=UnlessCondition(qt_scaling_fix)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'qt_scaling_fix',
            default_value='true',
            description='Corrige le clignotement de RViz sous Wayland (X11 + scaling Qt désactivé)'
        ),
        robot_state_publisher_node,
        rviz_node,
        rviz_node_no_fix
    ])
