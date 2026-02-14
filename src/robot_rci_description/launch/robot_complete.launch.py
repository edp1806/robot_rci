from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Chemin vers le display.launch.py existant
    description_pkg = get_package_share_directory('robot_rci_description')
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        )
    )

    # Node GUI
    gui_node = Node(
        package='robot_rci_gui',
        executable='control_panel',
        name='robot_control_gui',
        output='screen'
    )

    return LaunchDescription([
        display_launch,
        gui_node,
    ])
