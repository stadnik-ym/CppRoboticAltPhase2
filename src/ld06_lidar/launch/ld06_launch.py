#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LD06 lidar'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='230400',
        description='Serial baudrate'
    )

    ld06_node = Node(
        package='ld06_ros2',
        executable='ld06_node',
        name='ld06_node',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baudrate': LaunchConfiguration('baudrate')},
            {'frame_id': 'laser_frame'},
            {'publish_rate_hz': 50.0},
            {'angle_resolution_deg': 1.0},
            {'angle_offset_deg': 0.0},
            {'invert_angle_direction': False},
            {'front_min_deg': -35.0},
            {'front_max_deg': 35.0},
            {'point_max_age_sec': 0.35},
            {'front_min_points': 1},
        ]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        ld06_node,
    ])
