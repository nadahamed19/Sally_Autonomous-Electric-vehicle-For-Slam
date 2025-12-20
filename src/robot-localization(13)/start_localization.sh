#!/bin/bash
echo "🤖 Starting Robot Localization Node..."

cd ~/robot-localization_3

# Fix NumPy compatibility
export PYTHONPATH=$PYTHONPATH:$(pwd)

# Apply NumPy fix and run
python3 -c "
import numpy as np
if not hasattr(np, 'float'): 
    np.float = float
    print('✅ NumPy compatibility fix applied')

print('🚀 Starting localization node...')
from robot_localization.odom_tf_fusion_node import main
main()
"
