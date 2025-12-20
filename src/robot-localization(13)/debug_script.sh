#!/bin/bash

echo "=== Robot Localization Debug Script ==="
echo "This script will help diagnose launch issues"

# Check Python dependencies
echo -e "\n=== Checking Python dependencies ==="
for pkg in numpy tf_transformations
do
    python3 -c "import $pkg; print(f'$pkg: OK')" || echo "$pkg: MISSING"
done

# Check ROS2 dependencies
echo -e "\n=== Checking ROS2 dependencies ==="
for pkg in rclpy geometry_msgs nav_msgs tf2_ros tf2_msgs
do
    python3 -c "import $pkg; print(f'$pkg: OK')" 2>/dev/null || echo "$pkg: MISSING"
done

# Check package structure
echo -e "\n=== Checking package structure ==="
PKG_DIR=$(pwd)
echo "Current directory: $PKG_DIR"

if [ -f "$PKG_DIR/setup.py" ]; then
    echo "setup.py: OK"
else
    echo "setup.py: MISSING"
fi

if [ -f "$PKG_DIR/package.xml" ]; then
    echo "package.xml: OK"
else
    echo "package.xml: MISSING"
fi

if [ -d "$PKG_DIR/robot_localization" ]; then
    echo "robot_localization directory: OK"
else
    echo "robot_localization directory: MISSING"
fi

if [ -f "$PKG_DIR/robot_localization/__init__.py" ]; then
    echo "__init__.py: OK"
else
    echo "__init__.py: MISSING"
fi

if [ -f "$PKG_DIR/robot_localization/odom_tf_fusion_node.py" ]; then
    echo "odom_tf_fusion_node.py: OK"
else
    echo "odom_tf_fusion_node.py: MISSING"
fi

if [ -d "$PKG_DIR/resource" ]; then
    echo "resource directory: OK"
else
    echo "resource directory: MISSING"
fi

if [ -f "$PKG_DIR/resource/robot_localization" ]; then
    echo "resource marker: OK"
else
    echo "resource marker: MISSING"
fi

if [ -d "$PKG_DIR/launch" ]; then
    echo "launch directory: OK"
else
    echo "launch directory: MISSING"
fi

if [ -f "$PKG_DIR/launch/odom_tf_fusion_launch.py" ]; then
    echo "launch file: OK"
else
    echo "launch file: MISSING"
fi

# Try to run the node directly
echo -e "\n=== Trying to run the node directly ==="
echo "python3 $PKG_DIR/robot_localization/odom_tf_fusion_node.py"
python3 $PKG_DIR/robot_localization/odom_tf_fusion_node.py &
NODE_PID=$!
sleep 2
kill $NODE_PID 2>/dev/null

echo -e "\n=== Recommendations ==="
echo "1. Make sure all dependencies are installed:"
echo "   sudo apt install python3-numpy python3-tf-transformations ros-humble-tf2-ros"
echo "2. Try rebuilding the package:"
echo "   cd ~/Project_ws && colcon build --packages-select robot_localization --symlink-install"
echo "3. Source the workspace:"
echo "   source ~/Project_ws/install/setup.bash"
echo "4. Run the node directly first to check for errors:"
echo "   ros2 run robot_localization odom_tf_fusion_node"
echo "5. Then try the launch file:"
echo "   ros2 launch robot_localization odom_tf_fusion_launch.py"
