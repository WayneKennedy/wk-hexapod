from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hexapod_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wayne Kennedy',
    maintainer_email='wayne@zappfyre.com',
    description='Autonomous behavior system for Hexapod Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_manager = hexapod_autonomy.autonomy_manager:main',
            'slam_monitor = hexapod_autonomy.slam_monitor:main',
            'look_around = hexapod_autonomy.look_around:main',
            'frontier_explorer = hexapod_autonomy.frontier_explorer:main',
            'mission_server = hexapod_autonomy.mission_server:main',
        ],
    },
)
