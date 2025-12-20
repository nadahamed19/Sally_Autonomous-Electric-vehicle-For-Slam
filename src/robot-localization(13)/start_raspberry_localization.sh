#!/bin/bash
echo "🤖 Starting Raspberry Pi Localization Node..."

cd ~/robot-localization_1
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:$(pwd)

# NumPy fix and run
python3 -c "
import numpy as np
if not hasattr(np, 'float'): 
    np.float = float
    print('✅ NumPy compatibility fix applied')

print('🚀 Starting Raspberry Pi Localization Node...')
print('📡 Subscribing to: /wheel/odom, /imu_data')
print('📤 Publishing to: /localization/odometry, /localization/pose')
print('Press Ctrl+C to stop')
print()

from robot_localization.raspberry_localization_node import main
main()
"
