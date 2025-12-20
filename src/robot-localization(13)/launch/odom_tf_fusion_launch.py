#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for odom-tf fusion node"""
    
    # Odometry-TF fusion node
    odom_tf_fusion_node = Node(
        package='robot_localization',
        executable='odom_tf_fusion_node',
        name='odom_tf_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        odom_tf_fusion_node
    ])
