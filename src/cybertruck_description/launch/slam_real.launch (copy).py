#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # ═══════════════════════════════════════════════════════════
    # PATHS
    # ═══════════════════════════════════════════════════════════
    pkg_name = 'cybertruck_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'cybertruck.urdf')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # ═══════════════════════════════════════════════════════════
    # ARGUMENTS
    # ═══════════════════════════════════════════════════════════
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for real robot)'
    )
    
    # ═══════════════════════════════════════════════════════════
    # SENSOR NODES (على الـ Raspberry Pi)
    # ═══════════════════════════════════════════════════════════
    
    # IMU Node - بينشر على /imu_data
    imu_node = Node(
        package='my_sensor_pkg',
        executable='imu',
        name='imu_node',
        output='screen'
    )
    
    # Encoder Node - بينشر على /wheel_odom
    encoder_node = Node(
        package='my_sensor_pkg',
        executable='encoder',
        name='encoder_node',
        output='screen'
    )
    
    # ═══════════════════════════════════════════════════════════
    # LOCALIZATION NODE (EKF Fusion)
    # بياخد: /wheel_odom + /imu_data
    # بينشر: /localization/odometry + TF (odom→base_link)
    # ═══════════════════════════════════════════════════════════
    localization_node = Node(
        package='robot_localization',  # أو اسم الـ package بتاعك
        executable='raspberry_localization_node',
        name='raspberry_localization_node',
        output='screen',
        # Remap output to /odom for SLAM toolbox
        remappings=[
            ('/localization/odometry', '/odom'),  # ← مهم جداً!
        ]
    )
    
    # ═══════════════════════════════════════════════════════════
    # CONTROL NODE
    # ═══════════════════════════════════════════════════════════
    control_node = Node(
        package='control_node',
        executable='control_node',
        name='control_node',
        output='screen'
    )
    
    # ═══════════════════════════════════════════════════════════
    # RPLIDAR A1
    # ═══════════════════════════════════════════════════════════
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'inverted': False,
        }]
    )
    
    # ═══════════════════════════════════════════════════════════
    # ROBOT STATE PUBLISHER
    # ═══════════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # ═══════════════════════════════════════════════════════════
    # JOINT STATE PUBLISHER
    # ═══════════════════════════════════════════════════════════
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ═══════════════════════════════════════════════════════════
    # SLAM TOOLBOX
    # ═══════════════════════════════════════════════════════════
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ═══════════════════════════════════════════════════════════
    # RVIZ2
    # ═══════════════════════════════════════════════════════════
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # ═══════════════════════════════════════════════════════════
    # TELEOP
    # ═══════════════════════════════════════════════════════════
    teleop_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',
            )
        ]
    )
    
    # ═══════════════════════════════════════════════════════════
    # LAUNCH!
    # ═══════════════════════════════════════════════════════════
    return LaunchDescription([
        declare_use_sim_time,
        
        # Sensors
        imu_node,
        encoder_node,
        localization_node,
        control_node,
        
        # SLAM
        rplidar_node,
        robot_state_publisher,
        joint_state_publisher,
        slam_node,
        
        # Visualization
        rviz_node,
        teleop_node,
    ])
