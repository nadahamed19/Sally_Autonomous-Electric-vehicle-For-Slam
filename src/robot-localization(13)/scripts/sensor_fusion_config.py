#!/usr/bin/env python3

import numpy as np

class SensorFusionConfig:
    """Configuration parameters for sensor fusion"""
    
    # EKF Parameters
    EKF_CONFIG = {
        # Initial state covariance
        'initial_covariance': np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        
        # Process noise covariance
        'process_noise': np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1]),
        
        # Measurement noise for odometry (x, y, theta)
        'odom_noise': np.diag([0.05, 0.05, 0.02]),
        
        # Measurement noise for IMU (angular velocity)
        'imu_noise': np.diag([0.01]),
        
        # Update frequency (Hz)
        'update_frequency': 50.0
    }
    
    # Encoder fusion weights
    ENCODER_FUSION = {
        'encoder1_weight': 0.5,
        'encoder2_weight': 0.5,
        'position_trust': 0.8,
        'orientation_trust': 0.9
    }
    
    # IMU configuration
    IMU_CONFIG = {
        'angular_velocity_threshold': 0.01,  # rad/s
        'acceleration_threshold': 0.1,       # m/s²
        'gyro_bias_estimation': True,
        'accel_bias_estimation': True
    }
    
    # Outlier detection
    OUTLIER_DETECTION = {
        'enable': True,
        'position_threshold': 0.5,    # meters
        'orientation_threshold': 0.2, # radians
        'velocity_threshold': 2.0     # m/s
    }

def get_config():
    """Get sensor fusion configuration"""
    return SensorFusionConfig()
