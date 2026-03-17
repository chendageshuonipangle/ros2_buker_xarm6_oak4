from setuptools import setup
import os
from glob import glob

package_name = 'path_planning'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.algorithms'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc-24',
    maintainer_email='user@todo.todo',
    description='Custom path planning algorithms for Bunker Mini robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jps_planner_node = path_planning.jps_planner_node:main',
            'bunker_path_planner = path_planning.bunker_path_planner:main',
        ],
    },
)
