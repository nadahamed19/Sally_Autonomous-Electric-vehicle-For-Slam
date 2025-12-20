#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # First apply the NumPy fix, then start the node
    numpy_fix = ExecuteProcess(
        cmd=["python3", "-c", "import numpy as np; if not hasattr(np, 'float'): np.float = float"],
        output='screen'
    )
    
    # Localization node
    localization_node = Node(
        package='robot_localization',
        executable='odom_tf_fusion_node',
        name='odom_tf_fusion_node',
        output='screen',
        parameters=[{
            'odom_weight': 0.7,
            'tf_weight': 0.3,
        }]
    )
    
    return LaunchDescription([
        numpy_fix,
        localization_node
    ])
