#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package info
    pkg_name = 'cybertruck_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'cybertruck.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'cybertruck.rviz')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Arguments
    gui = LaunchConfiguration('gui', default='true')
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([
        declare_gui,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
