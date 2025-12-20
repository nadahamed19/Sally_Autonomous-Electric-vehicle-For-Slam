#!/bin/bash

echo "🤖 Starting Raspberry Pi Localization Node..."

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/robot-localization_1/install/setup.bash

# Run the node
ros2 run robot_localization raspberry_localization_node
