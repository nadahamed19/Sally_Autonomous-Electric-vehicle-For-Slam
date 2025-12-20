#!/bin/bash

echo "🚀 Launching Raspberry Pi Localization Node..."

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/robot-localization_1/install/setup.bash

# Launch the node
ros2 launch robot_localization raspberry_localization.launch.py
