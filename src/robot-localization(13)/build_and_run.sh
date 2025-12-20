#!/bin/bash

echo "🚀 Building and Running C++ Robot Localization Node"
echo "=================================================="

# Step 1: Navigate to your workspace
cd ~/robot-localization_1
echo "📁 Current directory: $(pwd)"

# Step 2: Clean up old Python files (if they exist)
echo "🧹 Cleaning up old Python files..."
rm -f setup.py
rm -rf robot_localization/
rm -rf build/ install/ log/

# Step 3: Install required dependencies
echo "📦 Installing dependencies..."
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs

# Step 4: Source ROS2
echo "🔧 Sourcing ROS2..."
source /opt/ros/humble/setup.bash

# Step 5: Build the package
echo "🔨 Building the C++ package..."
colcon build --packages-select robot_localization --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
else
    echo "❌ Build failed!"
    exit 1
fi

# Step 6: Source the workspace
echo "🔧 Sourcing workspace..."
source install/setup.bash

# Step 7: Check if executable exists
echo "🔍 Checking executables..."
ros2 pkg executables robot_localization

echo ""
echo "🎉 Setup complete! Now you can run:"
echo "   ros2 run robot_localization raspberry_localization_node"
echo "   OR"
echo "   ros2 launch robot_localization raspberry_localization.launch.py"
echo ""
echo "📡 The node will subscribe to:"
echo "   - /wheel_odom (nav_msgs/Odometry)"
echo "   - /imu_data (sensor_msgs/Imu)"
echo ""
echo "📤 The node will publish to:"
echo "   - /localization/odometry (nav_msgs/Odometry)"
echo "   - /localization/pose (geometry_msgs/PoseWithCovarianceStamped)"
