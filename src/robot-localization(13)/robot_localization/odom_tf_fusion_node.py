#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class OdomTFFusionNode(Node):
    """
    Localization node that fuses:
    - /odom (already fused dual encoder data)
    - /tf (transform data)
    
    Uses Extended Kalman Filter for optimal sensor fusion
    """
    
    def __init__(self):
        super().__init__('odom_tf_fusion_node')
        
        # EKF state: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1  # Covariance matrix
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])  # Process noise
        self.R_odom = np.diag([0.02, 0.02, 0.01])  # Odometry noise (lower since it's already fused)
        self.R_tf = np.diag([0.03, 0.03, 0.015])  # TF noise (slightly higher uncertainty)
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Your dual-encoder fused odometry
            self.odom_callback,
            self.sensor_qos
        )
        
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',  # Your transform data
            self.tf_callback,
            self.sensor_qos
        )
        
        # Publishers
        self.fused_odom_pub = self.create_publisher(
            Odometry,
            '/localization/odometry',
            self.reliable_qos
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localization/pose',
            self.reliable_qos
        )
        
        # TF broadcaster for output
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Data storage
        self.latest_odom = None
        self.latest_tf_transform = None
        self.last_update_time = self.get_clock().now()
        self.last_tf_time = None
        
        # Fusion parameters (adjustable)
        self.odom_weight = 0.7  # Higher weight for encoder data
        self.tf_weight = 0.3    # Lower weight for TF data
        
        # Timer for main fusion loop
        self.timer = self.create_timer(0.02, self.fusion_callback)  # 50 Hz
        
        self.get_logger().info('Odometry-TF Fusion Node initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /odom (nav_msgs/Odometry) - dual encoder fused data')
        self.get_logger().info('  - /tf (tf2_msgs/TFMessage) - transform data')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /localization/odometry (nav_msgs/Odometry)')
        self.get_logger().info('  - /localization/pose (geometry_msgs/PoseWithCovarianceStamped)')
        self.get_logger().info(f'Fusion weights: odom={self.odom_weight}, tf={self.tf_weight}')
    
    def odom_callback(self, msg):
        """Process odometry from dual encoder fusion"""
        self.latest_odom = msg
        
        self.get_logger().debug(
            f'Odom: pos=({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}), '
            f'vel=({msg.twist.twist.linear.x:.3f}, {msg.twist.twist.angular.z:.3f})',
            throttle_duration_sec=2.0
        )
    
    def tf_callback(self, msg):
        """Process TF messages - look for your specific transform"""
        # Look for transforms from Assem1/odom to Assem1/base_link (from your data)
        for transform in msg.transforms:
            if (transform.header.frame_id == 'Assem1/odom' and 
                transform.child_frame_id == 'Assem1/base_link'):
                
                self.latest_tf_transform = transform
                self.last_tf_time = self.get_clock().now()
                
                self.get_logger().debug(
                    f'TF: pos=({transform.transform.translation.x:.3f}, '
                    f'{transform.transform.translation.y:.3f}), '
                    f'frame={transform.header.frame_id} -> {transform.child_frame_id}',
                    throttle_duration_sec=2.0
                )
                break
    
    def predict(self, dt):
        """EKF prediction step"""
        if dt <= 0:
            return
            
        # State transition matrix
        F = np.eye(6)
        F[0, 3] = dt  # x += vx * dt
        F[1, 4] = dt  # y += vy * dt  
        F[2, 5] = dt  # theta += omega * dt
        
        # Predict state
        self.state = F @ self.state
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def fusion_callback(self):
        """Main fusion callback - combines odometry and TF data"""
        if self.latest_odom is None:
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if dt > 0:
            # Prediction step
            self.predict(dt)
            
            # Extract odometry data
            odom_x = self.latest_odom.pose.pose.position.x
            odom_y = self.latest_odom.pose.pose.position.y
            odom_q = self.latest_odom.pose.pose.orientation
            odom_euler = tf_transformations.euler_from_quaternion([odom_q.x, odom_q.y, odom_q.z, odom_q.w])
            odom_theta = odom_euler[2]
            
            # Extract velocity data
            odom_vx = self.latest_odom.twist.twist.linear.x
            odom_vy = self.latest_odom.twist.twist.linear.y
            odom_omega = self.latest_odom.twist.twist.angular.z
            
            # Update with odometry measurement
            self.update_odometry(odom_x, odom_y, odom_theta, odom_vx, odom_vy, odom_omega)
            
            # Update with TF data if available and recent
            if self.latest_tf_transform is not None and self.last_tf_time is not None:
                tf_age = (current_time - self.last_tf_time).nanoseconds / 1e9
                if tf_age < 0.5:  # Only use TF data if it's less than 500ms old
                    self.update_with_tf()
                else:
                    self.get_logger().debug('TF data too old, skipping TF update', throttle_duration_sec=5.0)
            
            # Publish fused results
            self.publish_fused_odometry()
            self.publish_pose()
            self.publish_transform()
            
            self.last_update_time = current_time
    
    def update_odometry(self, x, y, theta, vx, vy, omega):
        """Update EKF with odometry measurement"""
        # Measurement model for position, orientation, and velocities
        H = np.eye(6)  # Direct observation of all states
        
        z = np.array([x, y, theta, vx, vy, omega])
        y_k = z - H @ self.state
        y_k[2] = self.normalize_angle(y_k[2])  # Handle angle wrapping
        
        # Measurement noise covariance
        R_full = np.zeros((6, 6))
        R_full[:3, :3] = self.R_odom  # Position and orientation
        R_full[3:, 3:] = np.diag([0.05, 0.05, 0.02])  # Velocity measurements
        
        S = H @ self.P @ H.T + R_full
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y_k
        self.state[2] = self.normalize_angle(self.state[2])
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def update_with_tf(self):
        """Update EKF with TF transform data"""
        if self.latest_tf_transform is None:
            return
        
        # Extract TF data
        tf_x = self.latest_tf_transform.transform.translation.x
        tf_y = self.latest_tf_transform.transform.translation.y
        tf_q = self.latest_tf_transform.transform.rotation
        tf_euler = tf_transformations.euler_from_quaternion([tf_q.x, tf_q.y, tf_q.z, tf_q.w])
        tf_theta = tf_euler[2]
        
        # Measurement model for TF (position and orientation only)
        H_tf = np.zeros((3, 6))
        H_tf[0, 0] = 1  # x
        H_tf[1, 1] = 1  # y
        H_tf[2, 2] = 1  # theta
        
        z_tf = np.array([tf_x, tf_y, tf_theta])
        y_k_tf = z_tf - H_tf @ self.state
        y_k_tf[2] = self.normalize_angle(y_k_tf[2])  # Handle angle wrapping
        
        S_tf = H_tf @ self.P @ H_tf.T + self.R_tf
        K_tf = self.P @ H_tf.T @ np.linalg.inv(S_tf)
        
        self.state = self.state + K_tf @ y_k_tf
        self.state[2] = self.normalize_angle(self.state[2])
        self.P = (np.eye(6) - K_tf @ H_tf) @ self.P
        
        self.get_logger().debug(
            f'TF Update: tf_pos=({tf_x:.3f}, {tf_y:.3f}), '
            f'fused_pos=({self.state[0]:.3f}, {self.state[1]:.3f})',
            throttle_duration_sec=2.0
        )
    
    def publish_fused_odometry(self):
        """Publish fused odometry result"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position from EKF state
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = 0.0
        
        # Orientation from EKF state
        quat = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        # Velocity from EKF state
        msg.twist.twist.linear.x = self.state[3]
        msg.twist.twist.linear.y = self.state[4]
        msg.twist.twist.angular.z = self.state[5]
        
        # Covariance from EKF
        pose_cov = np.zeros(36)
        pose_cov[0] = self.P[0, 0]   # x variance
        pose_cov[7] = self.P[1, 1]   # y variance
        pose_cov[35] = self.P[2, 2]  # theta variance
        msg.pose.covariance = pose_cov.tolist()
        
        twist_cov = np.zeros(36)
        twist_cov[0] = self.P[3, 3]   # vx variance
        twist_cov[7] = self.P[4, 4]   # vy variance
        twist_cov[35] = self.P[5, 5]  # omega variance
        msg.twist.covariance = twist_cov.tolist()
        
        self.fused_odom_pub.publish(msg)
    
    def publish_pose(self):
        """Publish pose with covariance"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Position
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = 0.0
        
        # Orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        # Covariance
        pose_cov = np.zeros(36)
        pose_cov[0] = self.P[0, 0]
        pose_cov[7] = self.P[1, 1]
        pose_cov[35] = self.P[2, 2]
        msg.pose.covariance = pose_cov.tolist()
        
        self.pose_pub.publish(msg)
    
    def publish_transform(self):
        """Publish fused transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Translation
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        
        # Rotation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OdomTFFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
