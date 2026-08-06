from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'jetson_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='hillman.grant@ufl.edu',
    description='ROS2 bridge that consumes Jetson RTSP streams and publishes sensor_msgs/Image',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_bridge_node = jetson_bridge.camera_bridge_node:main',
        ],
    },
)
