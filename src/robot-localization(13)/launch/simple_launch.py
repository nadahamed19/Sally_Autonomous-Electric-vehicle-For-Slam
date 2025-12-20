#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Simple launch file that works reliably"""
    
    # Just launch the odom-tf fusion node
    node = Node(
        package='robot_localization',
        executable='odom_tf_fusion_node',
        name='odom_tf_fusion_node',
        output='screen'
    )
    
    return LaunchDescription([node])
