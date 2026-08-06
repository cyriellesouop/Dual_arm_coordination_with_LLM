from setuptools import find_packages, setup

package_name = 'kinova_pick_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='espasciani',
    maintainer_email='espasciani@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_planner = kinova_pick_planner.kinova_pick_planner:demo_scenario',
            'camera_pick = kinova_pick_planner.camera_integration_example:main',
            'plan_test = kinova_pick_planner.plan_test:main',
            'arm_controller = kinova_pick_planner.arm_controller:demo_with_simulated_camera',
            'plan_test6 = kinova_pick_planner.plan_test6:main',
            'pymoveit2_test = kinova_pick_planner.pymoveit2_test:main',

        ],
    },
)
