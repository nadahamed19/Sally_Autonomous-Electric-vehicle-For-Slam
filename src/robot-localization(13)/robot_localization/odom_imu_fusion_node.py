#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class OdomIMUFusionNode(Node):
    """
    Localization node that fuses:
    - /odom (already fused encoder data from your dual encoder node)
    - /imu/data (IMU sensor data)
    
    Uses Extended Kalman Filter for optimal sensor fusion
    """
    
    def __init__(self):
        super().__init__('odom_imu_fusion_node')
        
        # EKF state: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1  # Covariance matrix
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])  # Process noise
        self.R_odom = np.diag([0.02, 0.02, 0.01])  # Odometry noise (lower since it's already fused)
        self.R_imu = np.diag([0.005])  # IMU noise
        
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
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',  # Your IMU data
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
        self.last_update_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()
        
        # Timer for main fusion loop
        self.timer = self.create_timer(0.02, self.fusion_callback)  # 50 Hz
        
        self.get_logger().info('Odometry-IMU Fusion Node initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /odom (nav_msgs/Odometry) - dual encoder fused data')
        self.get_logger().info('  - /imu/data (sensor_msgs/Imu)')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /localization/odometry (nav_msgs/Odometry)')
        self.get_logger().info('  - /localization/pose (geometry_msgs/PoseWithCovarianceStamped)')
    
    def odom_callback(self, msg):
        """Process odometry from dual encoder fusion"""
        self.latest_odom = msg
        
        self.get_logger().debug(
            f'Odom: pos=({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}), '
            f'vel=({msg.twist.twist.linear.x:.3f}, {msg.twist.twist.angular.z:.3f})',
            throttle_duration_sec=2.0
        )
    
    def imu_callback(self, msg):
        """Process IMU data and update EKF prediction"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        
        if 0 < dt < 1.0:  # Sanity check
            # EKF Prediction step with IMU
            self.predict_with_imu(dt, msg)
            self.last_imu_time = current_time
            
            self.get_logger().debug(
                f'IMU: angular_vel={msg.angular_velocity.z:.3f}, '
                f'linear_accel=({msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f})',
                throttle_duration_sec=2.0
            )
    
    def predict_with_imu(self, dt, imu_msg):
        """EKF prediction step enhanced with IMU data"""
        # State transition matrix
        F = np.eye(6)
        F[0, 3] = dt  # x += vx * dt
        F[1, 4] = dt  # y += vy * dt  
        F[2, 5] = dt  # theta += omega * dt
        
        # Use IMU angular velocity for better prediction
        imu_omega = imu_msg.angular_velocity.z
        
        # Predict state with IMU angular velocity
        predicted_state = F @ self.state
        predicted_state[5] = imu_omega  # Use IMU angular velocity directly
        predicted_state[2] = self.state[2] + imu_omega * dt  # Update theta with IMU
        predicted_state[2] = self.normalize_angle(predicted_state[2])
        
        self.state = predicted_state
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
        # Update with IMU angular velocity measurement
        self.update_imu_angular_velocity(imu_omega)
    
    def update_imu_angular_velocity(self, angular_velocity):
        """Update EKF with IMU angular velocity measurement"""
        H = np.zeros((1, 6))
        H[0, 5] = 1  # omega measurement
        
        z = np.array([angular_velocity])
        y = z - H @ self.state
        
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def fusion_callback(self):
        """Main fusion callback - combines odometry and IMU-enhanced state"""
        if self.latest_odom is None:
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if dt > 0:
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
            
            # Update EKF with odometry measurement
            self.update_odometry(odom_x, odom_y, odom_theta, odom_vx, odom_vy, odom_omega)
            
            # Publish fused results
            self.publish_fused_odometry()
            self.publish_pose()
            self.publish_transform()
            
            self.last_update_time = current_time
    
    def update_odometry(self, x, y, theta, vx, vy, omega):
        """Update EKF with full odometry measurement"""
        # Measurement model for full state observation
        H = np.eye(6)  # Direct observation of all states
        
        z = np.array([x, y, theta, vx, vy, omega])
        y_k = z - H @ self.state
        y_k[2] = self.normalize_angle(y_k[2])  # Handle angle wrapping
        
        # Use full covariance for odometry + velocity
        R_full = np.zeros((6, 6))
        R_full[:3, :3] = self.R_odom  # Position and orientation
        R_full[3:, 3:] = np.diag([0.05, 0.05, 0.02])  # Velocity measurements
        
        S = H @ self.P @ H.T + R_full
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y_k
        self.state[2] = self.normalize_angle(self.state[2])
        self.P = (np.eye(6) - K @ H) @ self.P
        
        self.get_logger().debug(
            f'EKF State: pos=({self.state[0]:.3f}, {self.state[1]:.3f}), '
            f'theta={math.degrees(self.state[2]):.1f}°, '
            f'vel=({self.state[3]:.3f}, {self.state[5]:.3f})',
            throttle_duration_sec=1.0
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
        node = OdomIMUFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
