#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations
import math

class ExtendedKalmanFilter:
    def __init__(self):
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        
        # State covariance matrix
        self.P = np.eye(6) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise covariance for odometry
        self.R_odom = np.diag([0.05, 0.05, 0.02])
        
        # Measurement noise covariance for IMU
        self.R_imu = np.diag([0.01])
        
        self.last_time = None
    
    def predict(self, dt):
        """Prediction step of EKF"""
        if dt <= 0:
            return
            
        # State transition model
        F = np.eye(6)
        F[0, 3] = dt  # x += vx * dt
        F[1, 4] = dt  # y += vy * dt
        F[2, 5] = dt  # theta += omega * dt
        
        # Predict state
        self.state = F @ self.state
        
        # Normalize angle
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update_odometry(self, x, y, theta):
        """Update step with odometry measurement"""
        # Measurement model (direct observation of position and orientation)
        H = np.zeros((3, 6))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # theta
        
        # Measurement
        z = np.array([x, y, theta])
        
        # Innovation
        y_k = z - H @ self.state
        y_k[2] = self.normalize_angle(y_k[2])  # Normalize angle difference
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_odom
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y_k
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Update covariance
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P
    
    def update_imu(self, angular_velocity):
        """Update step with IMU measurement"""
        # Measurement model (direct observation of angular velocity)
        H = np.zeros((1, 6))
        H[0, 5] = 1  # omega
        
        # Measurement
        z = np.array([angular_velocity])
        
        # Innovation
        y_k = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_imu
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y_k
        
        # Update covariance
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # QoS profiles for different types of data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize EKF
        self.ekf = ExtendedKalmanFilter()
        
        # Publishers with reliable QoS
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/localization/odometry', 
            reliable_qos
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localization/pose',
            reliable_qos
        )
        
        # Subscribers with appropriate QoS
        self.encoder1_sub = self.create_subscription(
            Odometry,
            '/encoder1/odometry',
            self.encoder1_callback,
            sensor_qos
        )
        
        self.encoder2_sub = self.create_subscription(
            Odometry,
            '/encoder2/odometry',
            self.encoder2_callback,
            sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Storage for encoder data
        self.encoder1_data = None
        self.encoder2_data = None
        self.last_update_time = self.get_clock().now()
        
        # Timer for periodic updates
        self.timer = self.create_timer(0.02, self.update_localization)  # 50 Hz
        
        self.get_logger().info('Localization node initialized')
    
    def encoder1_callback(self, msg):
        """Callback for first encoder"""
        self.encoder1_data = msg
        self.get_logger().debug('Received encoder1 data')
    
    def encoder2_callback(self, msg):
        """Callback for second encoder"""
        self.encoder2_data = msg
        self.get_logger().debug('Received encoder2 data')
    
    def imu_callback(self, msg):
        """Callback for IMU data"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if dt > 0:
            # Predict step
            self.ekf.predict(dt)
            
            # Update with IMU angular velocity
            angular_velocity = msg.angular_velocity.z
            self.ekf.update_imu(angular_velocity)
            
            self.last_update_time = current_time
            self.get_logger().debug(f'Updated with IMU data, dt: {dt:.4f}')
    
    def fuse_encoder_data(self):
        """Fuse data from both encoders"""
        if self.encoder1_data is None or self.encoder2_data is None:
            return None, None, None
        
        # Simple fusion: average the positions and orientations
        x1 = self.encoder1_data.pose.pose.position.x
        y1 = self.encoder1_data.pose.pose.position.y
        q1 = self.encoder1_data.pose.pose.orientation
        
        x2 = self.encoder2_data.pose.pose.position.x
        y2 = self.encoder2_data.pose.pose.position.y
        q2 = self.encoder2_data.pose.pose.orientation
        
        # Average positions
        x_fused = (x1 + x2) / 2.0
        y_fused = (y1 + y2) / 2.0
        
        # Convert quaternions to euler angles and average
        euler1 = tf_transformations.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
        euler2 = tf_transformations.euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])
        
        # Average yaw angles (handling wrap-around)
        theta1 = euler1[2]
        theta2 = euler2[2]
        
        # Handle angle wrapping
        diff = theta2 - theta1
        if diff > math.pi:
            theta2 -= 2 * math.pi
        elif diff < -math.pi:
            theta2 += 2 * math.pi
        
        theta_fused = (theta1 + theta2) / 2.0
        
        return x_fused, y_fused, theta_fused
    
    def update_localization(self):
        """Main localization update loop"""
        # Fuse encoder data
        x, y, theta = self.fuse_encoder_data()
        
        if x is not None:
            # Update EKF with fused odometry
            self.ekf.update_odometry(x, y, theta)
            
            # Publish results
            self.publish_odometry()
            self.publish_pose()
            self.publish_transform()
    
    def publish_odometry(self):
        """Publish fused odometry"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = self.ekf.state[0]
        msg.pose.pose.position.y = self.ekf.state[1]
        msg.pose.pose.position.z = 0.0
        
        # Orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.ekf.state[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        # Velocity
        msg.twist.twist.linear.x = self.ekf.state[3]
        msg.twist.twist.linear.y = self.ekf.state[4]
        msg.twist.twist.angular.z = self.ekf.state[5]
        
        # Covariance (simplified)
        pose_cov = np.zeros(36)
        pose_cov[0] = self.ekf.P[0, 0]   # x
        pose_cov[7] = self.ekf.P[1, 1]   # y
        pose_cov[35] = self.ekf.P[2, 2]  # theta
        msg.pose.covariance = pose_cov.tolist()
        
        twist_cov = np.zeros(36)
        twist_cov[0] = self.ekf.P[3, 3]   # vx
        twist_cov[7] = self.ekf.P[4, 4]   # vy
        twist_cov[35] = self.ekf.P[5, 5]  # omega
        msg.twist.covariance = twist_cov.tolist()
        
        self.odom_pub.publish(msg)
    
    def publish_pose(self):
        """Publish pose with covariance"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Position
        msg.pose.pose.position.x = self.ekf.state[0]
        msg.pose.pose.position.y = self.ekf.state[1]
        msg.pose.pose.position.z = 0.0
        
        # Orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.ekf.state[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        # Covariance
        pose_cov = np.zeros(36)
        pose_cov[0] = self.ekf.P[0, 0]   # x
        pose_cov[7] = self.ekf.P[1, 1]   # y
        pose_cov[35] = self.ekf.P[2, 2]  # theta
        msg.pose.covariance = pose_cov.tolist()
        
        self.pose_pub.publish(msg)
    
    def publish_transform(self):
        """Publish transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Translation
        t.transform.translation.x = self.ekf.state[0]
        t.transform.translation.y = self.ekf.state[1]
        t.transform.translation.z = 0.0
        
        # Rotation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.ekf.state[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    localization_node = LocalizationNode()
    
    try:
        rclpy.spin(localization_node)
    except KeyboardInterrupt:
        pass
    finally:
        localization_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
