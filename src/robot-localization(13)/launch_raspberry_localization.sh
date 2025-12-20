#!/bin/bash
echo "🚀 Launching Raspberry Pi Localization Node..."

cd ~/robot-localization_1
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:$(pwd)

# Apply NumPy fix
python3 -c "import numpy as np; if not hasattr(np, 'float'): np.float = float"

# Launch the node using the launch file
ros2 launch launch/raspberry_localization.launch.py
