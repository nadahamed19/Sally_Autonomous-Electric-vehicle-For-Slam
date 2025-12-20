#!/bin/bash

echo "🔍 Debug Build Information"
echo "========================="

cd ~/robot-localization_1

echo "📁 Directory contents:"
ls -la

echo ""
echo "📄 Package files:"
if [ -f "package.xml" ]; then
    echo "✅ package.xml exists"
else
    echo "❌ package.xml missing"
fi

if [ -f "CMakeLists.txt" ]; then
    echo "✅ CMakeLists.txt exists"
else
    echo "❌ CMakeLists.txt missing"
fi

if [ -d "src" ]; then
    echo "✅ src directory exists"
    echo "   Contents: $(ls src/)"
else
    echo "❌ src directory missing"
fi

echo ""
echo "🔧 ROS2 Environment:"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"

echo ""
echo "📦 Dependencies check:"
dpkg -l | grep -E "(libeigen3-dev|ros-humble-tf2)" || echo "Some dependencies may be missing"

echo ""
echo "🔨 Attempting build with verbose output..."
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_localization --cmake-args -DCMAKE_BUILD_TYPE=Release --verbose
