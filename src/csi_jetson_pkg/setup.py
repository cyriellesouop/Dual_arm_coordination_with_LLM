from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'csi_jetson_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))), # copy launch files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))), # copy config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='hillman.grant@ufl.edu',
    description='CSI publisher for Jetson Nano over LAN using JPEG compression',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'csi_publisher_node  = csi_jetson_pkg.csi_publisher_node:main',
            'csi_subscriber_node = csi_jetson_pkg.csi_subscriber_node:main',
            'camera_bridge_node  = csi_jetson_pkg.camera_bridge_node:main',
        ],
    },
)
