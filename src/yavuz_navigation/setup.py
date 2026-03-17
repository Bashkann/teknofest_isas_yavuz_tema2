from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yavuz_navigation'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mission_node = yavuz_navigation.mission_state_machine:main',
            'pose_bridge = yavuz_navigation.pose_bridge:main',
            'trajectory_logger = yavuz_navigation.trajectory_logger:main',
        ],
    },
)
