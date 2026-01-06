from setuptools import find_packages, setup

package_name = 'hexapod_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hardware.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wayne',
    maintainer_email='wayne@example.com',
    description='Hardware interface for Freenove Big Hexapod Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = hexapod_hardware.servo_driver:main',
            'imu_driver = hexapod_hardware.imu_driver:main',
            'battery_monitor = hexapod_hardware.battery_monitor:main',
            'led_controller = hexapod_hardware.led_controller:main',
            'range_finder_driver = hexapod_hardware.range_finder_driver:main',
            'buzzer_controller = hexapod_hardware.buzzer_controller:main',
            'power_indicator = hexapod_hardware.power_indicator:main',
            'startup_sequence = hexapod_hardware.startup_sequence:main',
        ],
    },
)
