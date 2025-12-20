#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # ═══════════════════════════════════════════════════════════
    # PATHS
    # ═══════════════════════════════════════════════════════════
    pkg_name = 'cybertruck_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'cybertruck.urdf')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    
    # Debug prints
    print(f"\n{'='*50}")
    print(f"REAL ROBOT NAVIGATION")
    print(f"{'='*50}")
    print(f"Map file: {map_file} (exists: {os.path.exists(map_file)})")
    print(f"Nav2 params: {nav2_params} (exists: {os.path.exists(nav2_params)})")
    print(f"URDF: {urdf_file} (exists: {os.path.exists(urdf_file)})")
    print(f"{'='*50}\n")
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # ═══════════════════════════════════════════════════════════
    # ARGUMENTS
    # ═══════════════════════════════════════════════════════════
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # ← مهم! false للروبوت الحقيقي
        description='Use simulation clock (false for real robot)'
    )
    
    # ═══════════════════════════════════════════════════════════
    # REAL ROBOT SENSOR NODES
    # ═══════════════════════════════════════════════════════════
    
    # IMU Node
    imu_node = Node(
        package='my_sensor_pkg',
        executable='imu',
        name='imu_node',
        output='screen'
    )
    
    # Encoder Node
    encoder_node = Node(
        package='my_sensor_pkg',
        executable='encoder',
        name='encoder_node',
        output='screen'
    )
    
    # Localization Node (EKF - بيدمج IMU + Encoders)
    # بينشر: /odom topic + TF (odom → base_link)
    localization_node = Node(
        package='robot_localization',
        executable='raspberry_localization_node',
        name='raspberry_localization_node',
        output='screen',
        remappings=[
            ('/localization/odometry', '/odom'),
        ]
    )
    
    # Control Node (بيستقبل /cmd_vel وبيحرك الموتورات)
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
    # NAV2 STACK
    # ═══════════════════════════════════════════════════════════
    
    # Map Server (بعد 2 ثانية)
    map_server = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': map_file},
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # AMCL - Localization على الخريطة (بعد 3 ثواني)
    amcl = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    nav2_params,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # Planner Server (بعد 4 ثواني)
    planner_server = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[
                    nav2_params,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # Controller Server (بعد 4 ثواني)
    controller_server = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[
                    nav2_params,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # Behavior Server (بعد 4.5 ثانية)
    behavior_server = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[
                    nav2_params,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # BT Navigator (بعد 4.5 ثانية)
    bt_navigator = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[
                    nav2_params,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # Smoother Server (بعد 4.5 ثانية)
    smoother_server = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[
                    nav2_params,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # Velocity Smoother (بعد 4.5 ثانية) - مهم للروبوت الحقيقي!
    velocity_smoother = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[
                    nav2_params,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),           # Input from Nav2
                    ('cmd_vel_smoothed', 'cmd_vel'),     # Output to robot
                ]
            )
        ]
    )
    
    # Lifecycle Manager (بعد 5.5 ثانية)
    lifecycle_manager = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': [
                        'map_server',
                        'amcl',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'bt_navigator',
                        'smoother_server',
                        'velocity_smoother'
                    ]}
                ]
            )
        ]
    )
    
    # ═══════════════════════════════════════════════════════════
    # RVIZ2
    # ═══════════════════════════════════════════════════════════
    rviz_node = TimerAction(
        period=6.0,
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
    # LAUNCH!
    # ═══════════════════════════════════════════════════════════
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        
        # ═══ REAL ROBOT NODES ═══
        imu_node,
        encoder_node,
        localization_node,
        control_node,
        rplidar_node,
        
        # ═══ ROBOT DESCRIPTION ═══
        robot_state_publisher,
        joint_state_publisher,
        
        # ═══ NAV2 STACK ═══
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        smoother_server,
        velocity_smoother,
        lifecycle_manager,
        
        # ═══ VISUALIZATION ═══
        rviz_node,
    ])
