#!/usr/bin/env python3
"""
Quick runner for the localization node with NumPy compatibility
"""
import numpy as np

# Fix NumPy compatibility issue
if not hasattr(np, 'float'):
    np.float = float
    print("✅ NumPy compatibility fix applied")

# Now import and run
try:
    from robot_localization.odom_tf_fusion_node import main
    print("🚀 Starting Odom-TF Fusion Node...")
    main()
except KeyboardInterrupt:
    print("\n🛑 Node stopped by user")
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
