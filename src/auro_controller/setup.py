from setuptools import find_packages, setup

package_name = 'auro_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/controller.launch.py',
                                               'launch/system.launch.py']),
        ('share/' + package_name + '/config', ['config/controller_params.yaml',
                                               'config/controller.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AURO Team',
    maintainer_email='csshan3@gmail.com',
    description='Perception-to-LLM-to-Arm controller bridge for AURO',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'controller_node = auro_controller.controller_node:main',
            'mock_perception_pub = auro_controller.mock_perception_pub:main',
            'manual_select = auro_controller.manual_select:main',
        ],
    },
)
