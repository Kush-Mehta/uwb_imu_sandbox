from glob import glob

from setuptools import setup

package_name = 'uwb_imu_sandbox'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Parshva Patel',
    maintainer_email='parshvapatel1456@gmail.com',
    description='UWB-IMU Sandbox ROS 2 nodes and launch files',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'uwb_node = uwb_imu_sandbox.uwb_node:main',
            'imu_node = uwb_imu_sandbox.imu_node:main',
            'algorithm_node = uwb_imu_sandbox.algorithm_node:main',
            'demo_driver = uwb_imu_sandbox.demo_driver:main',
            'self_check = uwb_imu_sandbox.self_check:main',
        ],
    },
)
