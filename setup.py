from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pete',
    maintainer_email='pete@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_smoother = robot_obstacle_avoidance.path_smoother:main',
            'trajectory_generator = robot_obstacle_avoidance.trajectory_generator:main',
            'rpp_controller = robot_obstacle_avoidance.rpp_controller:main',
            'obstacle_avoidance = robot_obstacle_avoidance.obstacle_avoidance_node:main',
        ],
    },
)
