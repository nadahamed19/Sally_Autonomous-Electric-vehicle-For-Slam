#!/usr/bin/env python3

import subprocess
import sys
import os

def launch_localization_system():
    """Launch the complete localization system"""
    
    print("🤖 Starting Robot Localization System...")
    print("=" * 50)
    
    # Check if ROS2 is available
    try:
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True, check=True)
        print(f"✅ ROS2 detected: {result.stdout.strip()}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ ROS2 not found. Please install ROS2 first.")
        return False
    
    # Required packages
    required_packages = [
        'numpy',
        'scipy',
        'matplotlib',
        'tf-transformations'
    ]
    
    print("\n📦 Checking Python dependencies...")
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package.replace('-', '_'))
            print(f"✅ {package}")
        except ImportError:
            print(f"❌ {package}")
            missing_packages.append(package)
    
    if missing_packages:
        print(f"\n⚠️  Missing packages: {', '.join(missing_packages)}")
        print("Install them with: pip install " + " ".join(missing_packages))
        return False
    
    print("\n🚀 All dependencies satisfied!")
    
    # Instructions for running the localization node
    print("\n" + "=" * 50)
    print("📋 TO RUN THE LOCALIZATION SYSTEM:")
    print("=" * 50)
    
    print("\n1. Source your ROS2 workspace:")
    print("   source /opt/ros/humble/setup.bash")
    print("   source ~/your_ws/install/setup.bash")
    
    print("\n2. Run the localization node:")
    print("   python3 localization_node.py")
    
    print("\n3. In separate terminals, publish your sensor data:")
    print("   - Encoder 1: ros2 topic pub /encoder1/odometry nav_msgs/Odometry ...")
    print("   - Encoder 2: ros2 topic pub /encoder2/odometry nav_msgs/Odometry ...")
    print("   - IMU: ros2 topic pub /imu/data sensor_msgs/Imu ...")
    
    print("\n4. Monitor the output:")
    print("   ros2 topic echo /localization/odometry")
    print("   ros2 topic echo /localization/pose")
    
    print("\n5. Visualize in RViz:")
    print("   rviz2")
    print("   Add displays for:")
    print("   - Odometry (/localization/odometry)")
    print("   - PoseWithCovariance (/localization/pose)")
    print("   - TF (to see transforms)")
    
    print("\n" + "=" * 50)
    print("🔧 CONFIGURATION TIPS:")
    print("=" * 50)
    
    print("\n• Adjust noise parameters in SensorFusionConfig")
    print("• Tune EKF parameters based on your robot's characteristics")
    print("• Monitor diagnostics for performance evaluation")
    print("• Use the diagnostics module to analyze localization accuracy")
    
    print("\n" + "=" * 50)
    print("📊 TOPICS PUBLISHED:")
    print("=" * 50)
    print("• /localization/odometry (nav_msgs/Odometry)")
    print("• /localization/pose (geometry_msgs/PoseWithCovarianceStamped)")
    print("• /tf (geometry_msgs/TransformStamped)")
    
    print("\n" + "=" * 50)
    print("📡 TOPICS SUBSCRIBED:")
    print("=" * 50)
    print("• /encoder1/odometry (nav_msgs/Odometry)")
    print("• /encoder2/odometry (nav_msgs/Odometry)")
    print("• /imu/data (sensor_msgs/Imu)")
    
    return True

if __name__ == "__main__":
    success = launch_localization_system()
    if success:
        print("\n🎉 System ready to launch!")
    else:
        print("\n💥 Setup incomplete. Please resolve the issues above.")
        sys.exit(1)
