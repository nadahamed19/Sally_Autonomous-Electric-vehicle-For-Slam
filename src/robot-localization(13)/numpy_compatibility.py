#!/usr/bin/env python3
"""
Compatibility layer for NumPy to handle deprecated np.float attribute
"""
import numpy as np

# Add back np.float for backward compatibility
if not hasattr(np, 'float'):
    np.float = float

print("NumPy compatibility layer loaded")
