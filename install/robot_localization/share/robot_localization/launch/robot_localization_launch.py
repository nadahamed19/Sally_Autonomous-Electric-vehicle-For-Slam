from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch arguments
    use_standard_node_arg = DeclareLaunchArgument(
        'use_standard_node',
        default_value='true',
        description='Use standard localization node (true) or dual encoder node (false)'
    )
    
    # Standard localization node (subscribes to /odom and /tf)
    standard_node = Node(
        package='robot_localization',
        executable='standard_localization_node',
        name='standard_localization_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_standard_node'))
    )
    
    # Dual encoder localization node
    dual_encoder_node = Node(
        package='robot_localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{
            'update_frequency': 50.0,
        }],
        remappings=[
            ('/encoder1/odometry', '/left_encoder'),
            ('/encoder2/odometry', '/right_encoder'),
        ],
        condition=UnlessCondition(LaunchConfiguration('use_standard_node'))
    )
    
    return LaunchDescription([
        use_standard_node_arg,
        standard_node,
        dual_encoder_node
    ])
