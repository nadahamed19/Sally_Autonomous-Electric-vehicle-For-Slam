#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    SetEnvironmentVariable,
    TimerAction,
    LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = 'cybertruck_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # File paths
    world_file = os.path.join(pkg_share, 'worlds', 'cybertruck_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'cybertruck.urdf')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rvizz', 'navigation.rviz')
    models_path = os.path.join(pkg_share, 'models')
    
    print(f"\n=== PATHS ===")
    print(f"Map file: {map_file}")
    print(f"Map exists: {os.path.exists(map_file)}")
    print(f"Nav2 params: {nav2_params}")
    print(f"Nav2 params exists: {os.path.exists(nav2_params)}")
    print(f"RViz config: {rviz_config}")
    print(f"RViz exists: {os.path.exists(rviz_config)}\n")
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    # Environment setup for Gazebo
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_path + ':' + pkg_share]
    )
    
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path + ':' + pkg_share]
    )
    
    # Ignition Gazebo
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '2', world_file],
        output='screen',
        additional_env={
            'IGN_GAZEBO_RESOURCE_PATH': models_path + ':' + pkg_share,
            'GZ_SIM_RESOURCE_PATH': models_path + ':' + pkg_share
        }
    )
    
    # Robot State Publisher
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
    
    # ROS-Ignition Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/lidar', '/scan'),
        ]
    )
    # Static TF: base_link -> Lidar (using same format as slam.launch.py)
    static_tf_lidar_to_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_to_scan',
        arguments=['0', '0', '0.02', '0', '0', '0', 'Lidar', 'cybertruck/Lidar/lidar']
    )
    
    # REMOVED: static_tf_map - AMCL will publish map->odom transform!
    
    # Map Server - starts after 2 seconds
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
    
    # AMCL - starts after 3 seconds
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
                ],
                remappings=[
                    ('/scan', '/scan'),
                ]
            )
        ]
    )
    
    # Planner Server - starts after 4 seconds
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
    
    # Controller Server - starts after 4 seconds
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
                ],
                remappings=[
                    ('/cmd_vel', '/cmd_vel')
                ]
            )
        ]
    )
    
    # Behavior Server - starts after 4.5 seconds
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
    
    # BT Navigator - starts after 4.5 seconds
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
    
    # Smoother Server - starts after 4.5 seconds
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
    
    # Nav2 Lifecycle Manager - starts after 5.5 seconds to manage all nodes
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
                        'smoother_server'
                    ]}
                ]
            )
        ]
    )
    
    # RViz - starts after 7 seconds
    rviz_node = TimerAction(
        period=3.0,
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
    
    return LaunchDescription([
        # Environment
        gz_resource_path,
        gz_sim_resource_path,
        # Arguments
        declare_use_sim_time,
        # Core simulation nodes (start immediately)
        ignition_gazebo,
        robot_state_publisher,
        bridge,
        static_tf_lidar_to_scan,
        # Nav2 nodes (with delays)
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        smoother_server,
        lifecycle_manager,
        # UI
        rviz_node,
    ])