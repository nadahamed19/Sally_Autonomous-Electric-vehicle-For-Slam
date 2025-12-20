#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class StandardLocalizationNode(Node):
    """
    Localization node that subscribes to standard ROS2 topics:
    - /odom (nav_msgs/Odometry)
    - /tf (tf2_msgs/TFMessage) 
    - /imu/data (sensor_msgs/Imu)
    """
    
    def __init__(self):
        super().__init__('standard_localization_node')
        
        # EKF state: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1  # Covariance matrix
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])  # Process noise
        self.R_odom = np.diag([0.05, 0.05, 0.02])  # Odometry noise
        self.R_imu = np.diag([0.01])  # IMU noise
        
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
        
        # Subscribers for your exact topics
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Your odometry topic
            self.odom_callback,
            self.sensor_qos
        )
        
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',  # Your transform topic
            self.tf_callback,
            self.sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
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
        
        # Timer for fusion and publishing
        self.timer = self.create_timer(0.02, self.fusion_callback)  # 50 Hz
        
        self.get_logger().info('Standard Localization Node initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /odom (nav_msgs/Odometry)')
        self.get_logger().info('  - /tf (tf2_msgs/TFMessage)')
        self.get_logger().info('  - /imu/data (sensor_msgs/Imu)')
    
    def odom_callback(self, msg):
        """Process odometry messages like your data"""
        self.latest_odom = msg
        
        # Log the received data format (matches your example)
        self.get_logger().debug(
            f'Odom: pos=({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}), '
            f'frame={msg.header.frame_id} -> {msg.child_frame_id}',
            throttle_duration_sec=2.0
        )
    
    def tf_callback(self, msg):
        """Process TF messages like your data"""
        # Look for transforms from Assem1/odom to Assem1/base_link
        for transform in msg.transforms:
            if (transform.header.frame_id == 'Assem1/odom' and 
                transform.child_frame_id == 'Assem1/base_link'):
                
                self.latest_tf_transform = transform
                
                # Log the received transform (matches your example)
                self.get_logger().debug(
                    f'TF: pos=({transform.transform.translation.x:.3f}, '
                    f'{transform.transform.translation.y:.3f}), '
                    f'frame={transform.header.frame_id} -> {transform.child_frame_id}',
                    throttle_duration_sec=2.0
                )
                break
    
    def imu_callback(self, msg):
        """Process IMU data and update EKF prediction"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if 0 < dt < 1.0:  # Sanity check
            # EKF Prediction step
            self.predict(dt)
            
            # Update with IMU angular velocity
            self.update_imu(msg.angular_velocity.z)
            
            self.last_update_time = current_time
    
    def predict(self, dt):
        """EKF prediction step"""
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
    
    def update_imu(self, angular_velocity):
        """Update EKF with IMU angular velocity"""
        H = np.zeros((1, 6))
        H[0, 5] = 1  # omega measurement
        
        z = np.array([angular_velocity])
        y = z - H @ self.state
        
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def fusion_callback(self):
        """Main fusion callback - combines odom and TF data"""
        if self.latest_odom is None:
            return
        
        # Extract odometry data
        odom_x = self.latest_odom.pose.pose.position.x
        odom_y = self.latest_odom.pose.pose.position.y
        odom_q = self.latest_odom.pose.pose.orientation
        odom_euler = tf_transformations.euler_from_quaternion([odom_q.x, odom_q.y, odom_q.z, odom_q.w])
        odom_theta = odom_euler[2]
        
        # Use TF data if available for fusion
        if self.latest_tf_transform is not None:
            tf_x = self.latest_tf_transform.transform.translation.x
            tf_y = self.latest_tf_transform.transform.translation.y
            tf_q = self.latest_tf_transform.transform.rotation
            tf_euler = tf_transformations.euler_from_quaternion([tf_q.x, tf_q.y, tf_q.z, tf_q.w])
            tf_theta = tf_euler[2]
            
            # Weighted fusion (you can adjust these weights)
            w_odom = 0.7  # Weight for odometry
            w_tf = 0.3    # Weight for TF
            
            # Fuse positions
            x_fused = w_odom * odom_x + w_tf * tf_x
            y_fused = w_odom * odom_y + w_tf * tf_y
            
            # Fuse orientations (handle angle wrapping)
            diff = tf_theta - odom_theta
            if diff > math.pi:
                tf_theta -= 2 * math.pi
            elif diff < -math.pi:
                tf_theta += 2 * math.pi
            
            theta_fused = w_odom * odom_theta + w_tf * tf_theta
            
            self.get_logger().debug(
                f'Fusion: odom=({odom_x:.3f},{odom_y:.3f}), tf=({tf_x:.3f},{tf_y:.3f}), '
                f'fused=({x_fused:.3f},{y_fused:.3f})',
                throttle_duration_sec=5.0
            )
        else:
            # Use only odometry if TF not available
            x_fused = odom_x
            y_fused = odom_y
            theta_fused = odom_theta
        
        # Update EKF with fused measurement
        self.update_position(x_fused, y_fused, theta_fused)
        
        # Publish fused results
        self.publish_fused_odometry()
        self.publish_pose()
        self.publish_transform()
    
    def update_position(self, x, y, theta):
        """Update EKF with position measurement"""
        H = np.zeros((3, 6))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # theta
        
        z = np.array([x, y, theta])
        y_k = z - H @ self.state
        y_k[2] = self.normalize_angle(y_k[2])  # Handle angle wrapping
        
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y_k
        self.state[2] = self.normalize_angle(self.state[2])
        self.P = (np.eye(6) - K @ H) @ self.P
    
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
        node = StandardLocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
