#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('robot_localization')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    # Localization node
    localization_node = Node(
        package='robot_localization',
        executable='localization_node.py',
        name='localization_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            os.path.join(pkg_dir, 'config', 'localization_params.yaml')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Remap topics if needed
            # ('/encoder1/odometry', '/your_robot/encoder1/odom'),
            # ('/encoder2/odometry', '/your_robot/encoder2/odom'),
            # ('/imu/data', '/your_robot/imu/data'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        LogInfo(msg="Starting Robot Localization System..."),
        localization_node,
    ])
