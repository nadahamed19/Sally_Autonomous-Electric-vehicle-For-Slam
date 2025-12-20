from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch argument to choose which node to run
    node_type_arg = DeclareLaunchArgument(
        'node_type',
        default_value='raspberry_localization_node',
        description='Which localization node to run',
        choices=['raspberry_localization_node', 'localization_node', 'standard_localization_node', 
                'odom_imu_fusion_node', 'odom_tf_fusion_node', 'ros2_localization_node']
    )
    
    # Localization node
    localization_node = Node(
        package='robot_localization',
        executable=LaunchConfiguration('node_type'),
        name=LaunchConfiguration('node_type'),
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        node_type_arg,
        LogInfo(msg=["🚀 Starting ", LaunchConfiguration('node_type'), "..."]),
        localization_node
    ])
