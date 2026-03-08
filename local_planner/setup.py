import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'local_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # RViz configs
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Local motion planning with DWA and APF for TurtleBot3',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dwa_planner = local_planner.dwa_planner:main',
            'apf_planner = local_planner.apf_planner:main',
            'dynamic_obstacle_manager = local_planner.dynamic_obstacle_manager:main',
        ],
    },
)
