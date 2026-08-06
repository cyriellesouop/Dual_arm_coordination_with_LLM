from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'camera_calibration'

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
    description='Camera calibration package for a configurable number of camera streams.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'intrinsic_checkerboard_calibration_node = camera_calibration.intrinsic_checkerboard_calibration_node:main',
            'intrinsic_aruco_calibration_node = camera_calibration.intrinsic_aruco_calibration_node:main',
            'stereo_aruco_calibration_node = camera_calibration.stereo_aruco_calibration_node:main',
        ],
    },
)
