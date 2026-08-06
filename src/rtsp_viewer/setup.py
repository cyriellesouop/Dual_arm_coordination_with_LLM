import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rtsp_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='hillman.grant@ufl.edu',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rtsp_viewer_node = rtsp_viewer.rtsp_viewer_node:main',
        ],
    },
)
