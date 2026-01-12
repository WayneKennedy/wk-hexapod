from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hexapod_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wayne',
    maintainer_email='wayne@example.com',
    description='Camera and face recognition nodes for Hexapod Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = hexapod_perception.camera_node:main',
            'face_recognition_node = hexapod_perception.face_recognition_node:main',
            'web_dashboard = hexapod_perception.web_dashboard:main',
        ],
    },
)
