from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stretch4_line_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ola Ghattas',
    maintainer_email='oghattas@hello-robot.com',
    description='ROS 2 bridge for Stretch 4 base line sensors via stretch4_body RobotClient',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_sensor_node = stretch4_line_sensor.line_sensor_node:main',
        ],
    },
)
