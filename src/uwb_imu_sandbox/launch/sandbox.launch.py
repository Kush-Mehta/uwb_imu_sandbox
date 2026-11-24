"""Start the UWB-IMU Sandbox.

Examples
--------
ros2 launch uwb_imu_sandbox sandbox.launch.py                (Gazebo window)
ros2 launch uwb_imu_sandbox sandbox.launch.py gui:=false     (headless)
ros2 launch uwb_imu_sandbox sandbox.launch.py demo:=true     (robot drives a loop)
ros2 launch uwb_imu_sandbox sandbox.launch.py udp_forward:=true udp_port:=9999
"""
import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Gazebo, the bridge and the sandbox nodes."""
    pkg = get_package_share_directory('uwb_imu_sandbox')
    params = os.path.join(pkg, 'config', 'sandbox.yaml')
    bridge = os.path.join(pkg, 'config', 'bridge.yaml')

    # Every launch gets its own Gazebo partition, so a Gazebo server left over
    # from an earlier run (Ctrl-C does not always stop it) cannot leak into this
    # one through the bridge. Override with gz_partition:=<name> if you want to
    # use `gz topic` from another terminal (export GZ_PARTITION=<name> there).
    default_partition = f'uwb_sandbox_{socket.gethostname()}_{os.getpid()}'
    partition = LaunchConfiguration('gz_partition')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    gz_args = PythonExpression([
        "'-r -v 1 ' + ('' if '", gui, "' == 'true' else '-s ') + '", world, "'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='show the Gazebo window'),
        DeclareLaunchArgument('world', default_value='uwb_warehouse.sdf',
                              description='world file'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='drive the robot around a loop'),
        DeclareLaunchArgument('algorithm', default_value='true',
                              description='start the empty algorithm node'),
        DeclareLaunchArgument('udp_forward', default_value='false',
                              description='forward UWB results over UDP'),
        DeclareLaunchArgument('udp_port', default_value='9999'),
        DeclareLaunchArgument('gz_partition', default_value=default_partition,
                              description='Gazebo transport partition of this run'),
        SetEnvironmentVariable('GZ_PARTITION', partition),
        # keep Qt's harmless GUI warnings out of the terminal
        SetEnvironmentVariable('QT_LOGGING_RULES', '*.debug=false;*.warning=false'),
        LogInfo(msg=['Gazebo partition of this run: ', partition]),
        gazebo,
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sandbox_bridge',
             parameters=[{'config_file': bridge, 'use_sim_time': True}], output='screen'),
        Node(package='uwb_imu_sandbox', executable='uwb_node', name='uwb_node', output='screen',
             parameters=[params, {'udp_forward': LaunchConfiguration('udp_forward'),
                                  'udp_port': LaunchConfiguration('udp_port')}]),
        Node(package='uwb_imu_sandbox', executable='imu_node', name='imu_node', output='screen',
             parameters=[params]),
        Node(package='uwb_imu_sandbox', executable='algorithm_node', name='algorithm_node',
             output='screen', parameters=[params],
             condition=IfCondition(LaunchConfiguration('algorithm'))),
        Node(package='uwb_imu_sandbox', executable='demo_driver', name='demo_driver',
             output='screen', parameters=[params],
             condition=IfCondition(LaunchConfiguration('demo'))),
    ])
