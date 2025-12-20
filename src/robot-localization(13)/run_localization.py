#!/usr/bin/env python3
"""
Runner script for robot localization with NumPy compatibility fix
"""
import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(__file__))

# Apply NumPy compatibility fix
import numpy_compatibility

# Now import and run the node
from robot_localization.odom_tf_fusion_node import main

if __name__ == '__main__':
    print("Starting Odom-TF Fusion Node...")
    try:
        main()
    except KeyboardInterrupt:
        print("\nNode stopped by user")
    except Exception as e:
        print(f"Error: {e}")
