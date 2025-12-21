#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package info
    pkg_name = 'cybertruck_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'cybertruck_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'cybertruck.urdf')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_share, 'rvizz', 'slam.rviz')
    models_path = os.path.join(pkg_share, 'models')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Set Gazebo resource paths
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_path + ':' + pkg_share]
    )
    
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path + ':' + pkg_share]
    )
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
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
    
    # Static TF: Lidar -> cybertruck/Lidar/lidar (CRITICAL!)
    static_tf_lidar_to_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_to_scan',
        arguments=['0', '0', '0.02', '0', '0', '0', 'Lidar', 'cybertruck/Lidar/lidar']
    )
    
    # SLAM Toolbox Node
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
    
    # RViz for SLAM (delayed start)
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
    
    # Teleop Keyboard (delayed start)
    teleop_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',
                remappings=[('/cmd_vel', '/cmd_vel')]
            )
        ]
    )
    
    return LaunchDescription([
        # Environment
        gz_resource_path,
        gz_sim_resource_path,
        # Arguments
        declare_use_sim_time,
        # Nodes
        ignition_gazebo,
        robot_state_publisher,
        bridge,
        static_tf_lidar_to_scan,
        slam_node,
        rviz_node,
        teleop_node,
    ])