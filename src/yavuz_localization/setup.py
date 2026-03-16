from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yavuz_localization'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Fatih ŞAHİN',
    entry_points={
        'console_scripts': [
            'pose_estimator = yavuz_localization.pose_estimator:main',
        ],
    },
)
