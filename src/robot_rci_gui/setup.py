from setuptools import setup

package_name = 'robot_rci_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Votre Nom',
    maintainer_email='votre.email@exemple.com',
    description='Interface graphique pour le robot RCI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_panel = robot_rci_gui.control_panel:main',
        ],
    },
)
