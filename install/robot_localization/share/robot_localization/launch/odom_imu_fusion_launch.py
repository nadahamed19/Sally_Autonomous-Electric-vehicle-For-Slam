from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Odometry-IMU fusion node
    odom_imu_fusion_node = Node(
        package='robot_localization',
        executable='odom_imu_fusion_node',
        name='odom_imu_fusion_node',
        output='screen',
        parameters=[{
            'update_frequency': 50.0,
        }]
    )
    
    return LaunchDescription([
        odom_imu_fusion_node
    ])
