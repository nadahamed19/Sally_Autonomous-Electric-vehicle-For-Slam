#!/bin/bash

echo "🚀 Simple Build Script for Robot Localization"
echo "============================================"

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the package
echo "🔨 Building the package..."
colcon build --packages-select robot_localization

# Source the workspace
echo "🔧 Sourcing the workspace..."
source install/setup.bash

echo "✅ Done! Now you can run:"
echo "ros2 launch robot_localization raspberry_localization.launch.py"
