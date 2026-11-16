# Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).
"""Launch the MPU6050 driver and, optionally, the base_link -> imu_link transform."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('mpu6050_driver'), 'config', 'mpu6050.yaml')

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML file with the driver parameters'),
        DeclareLaunchArgument(
            'publish_tf', default_value='false',
            description='Publish a static transform from parent_frame to imu_link '
                        '(set false if your URDF / robot_state_publisher already does)'),
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('x', default_value='0.0', description='IMU position [m]'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0', description='IMU rotation [rad]'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    driver = Node(
        package='mpu6050_driver',
        executable='mpu6050_node',
        name='mpu6050',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file'),
                    {'frame_id': LaunchConfiguration('imu_frame')}],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        condition=IfCondition(LaunchConfiguration('publish_tf')),
        arguments=[
            '--x', LaunchConfiguration('x'), '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--roll', LaunchConfiguration('roll'), '--pitch', LaunchConfiguration('pitch'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', LaunchConfiguration('parent_frame'),
            '--child-frame-id', LaunchConfiguration('imu_frame'),
        ],
    )

    return LaunchDescription(args + [driver, static_tf])
