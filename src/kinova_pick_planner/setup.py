from setuptools import find_packages, setup

package_name = 'kinova_pick_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='espasciani',
    maintainer_email='espasciani@todo.todo',
    description='Pick-and-place controller for Kinova Gen3 Lite with MoveIt2 and LLM integration',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'arm_controller = kinova_pick_planner.arm_controller:main',
            'arm_controller_ros = kinova_pick_planner.arm_controller_ros:main',
            'pick_planner = kinova_pick_planner.kinova_pick_planner:demo_scenario',
            'arm_test = kinova_pick_planner.arm_controller:standalone_test',
            'arm_controller_demo = kinova_pick_planner.arm_controller_DUMMY:demo_with_simulated_camera',
            'failure_monitor = kinova_pick_planner.failure_monitor:main',
            'dual_arm_coordinator = kinova_pick_planner.dual_arm_coordinator:main',
        ],
    },
)
