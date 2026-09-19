from setuptools import find_packages, setup

package_name = 'hexapod_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/body_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/controller.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Wayne Kennedy',
    maintainer_email='wayne@zappfyre.com',
    description='Body-centric hexapod controller with IK, gait generation, and IMU stabilization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = hexapod_controller.controller:main',
            'head_controller = hexapod_controller.head_controller:main',
        ],
    },
)
