from setuptools import setup
import os
from glob import glob

package_name = 'robot_rci_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Votre Nom',
    maintainer_email='votre.email@exemple.com',
    description='Nœuds de contrôle pour le robot RCI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mgd_node = robot_rci_control.mgd_node:main',
            'mgi_node = robot_rci_control.mgi_node:main',
            'trajectory_node = robot_rci_control.trajectory_node:main',
            'validation_node = robot_rci_control.validation_node:main',
            'workspace_publisher = robot_rci_control.workspace_publisher:main',
        ],
    },
)
