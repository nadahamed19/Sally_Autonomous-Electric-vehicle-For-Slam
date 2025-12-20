"""NumPy compatibility fix for older tf_transformations"""
import numpy as np

# Fix NumPy compatibility
if not hasattr(np, 'float'):
    np.float = float
    print("Applied NumPy compatibility fix")
