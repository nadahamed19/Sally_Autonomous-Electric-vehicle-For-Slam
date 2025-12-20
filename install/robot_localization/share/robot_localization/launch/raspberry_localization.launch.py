from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """Generate launch description for the C++ raspberry localization node"""
    
    # Raspberry Pi localization node
    localization_node = Node(
        package='robot_localization',
        executable='raspberry_localization_node',
        name='raspberry_localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_weight': 0.7,
            'tf_weight': 0.3,
        }]
    )
    
    return LaunchDescription([
        LogInfo(msg="🚀 Starting Raspberry Pi Localization Node (C++)..."),
        LogInfo(msg="📡 Subscribing to: /wheel_odom, /imu_data"),
        LogInfo(msg="📤 Publishing to: /localization/odometry, /localization/pose"),
        localization_node
    ])
