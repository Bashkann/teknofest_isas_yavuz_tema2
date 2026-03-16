from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yavuz_navigation'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Ahmet Kaan BAŞKAN',
    entry_points={
        'console_scripts': [
            'mission_node = yavuz_navigation.mission_state_machine:main',
        ],
    },
)
