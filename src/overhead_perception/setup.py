from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'overhead_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'calibration'),
            glob(os.path.join('..', '..', 'calibration', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='hillman.grant@ufl.edu',
    description='Multi-camera overhead perception: bridge, detection, and 3-D localisation',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'detector_node         = overhead_perception.detector_node:main',
            'object_localizer_node = overhead_perception.object_localizer_node:main',
            'aruco_localizer_node  = overhead_perception.aruco_localizer_node:main',
            'debug_overlay_node    = overhead_perception.debug_overlay_node:main',
        ],
    },
)
