#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class ROS2LocalizationNode(Node):
    """
    ROS2-optimized localization node with parameter server integration
    """
    
    def __init__(self):
        super().__init__('ros2_localization_node')
        
        # Declare parameters with descriptions
        self.declare_parameters()
        
        # Initialize EKF with parameters
        self.initialize_ekf()
        
        # Setup QoS profiles
        self.setup_qos_profiles()
        
        # Create publishers and subscribers
        self.setup_communication()
        
        # Initialize state
        self.odom_data = None
        self.last_update_time = self.get_clock().now()
        
        # Create timer for main loop
        update_freq = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/update_freq, self.update_callback)
        
        self.get_logger().info('ROS2 Localization Node initialized successfully')
    
    def declare_parameters(self):
        """Declare all ROS2 parameters with descriptions"""
        
        # Update frequency
        self.declare_parameter('update_frequency', 50.0,
            ParameterDescriptor(description='Localization update frequency in Hz'))
        
        # Frame IDs
        self.declare_parameter('odom_frame_id', 'odom',
            ParameterDescriptor(description='Odometry frame ID'))
        self.declare_parameter('base_frame_id', 'base_link',
            ParameterDescriptor(description='Base link frame ID'))
        
        # EKF parameters
        self.declare_parameter('initial_covariance', [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            ParameterDescriptor(description='Initial state covariance diagonal'))
        self.declare_parameter('process_noise', [0.01, 0.01, 0.01, 0.1, 0.1, 0.1],
            ParameterDescriptor(description='Process noise covariance diagonal'))
        
        # Measurement noise
        self.declare_parameter('odom_noise', [0.05, 0.05, 0.02],
            ParameterDescriptor(description='Odometry measurement noise'))
        self.declare_parameter('imu_noise', [0.01],
            ParameterDescriptor(description='IMU measurement noise'))
        
        # Encoder fusion
        self.declare_parameter('encoder1_weight', 0.5,
            ParameterDescriptor(description='Weight for encoder 1 in fusion'))
        self.declare_parameter('encoder2_weight', 0.5,
            ParameterDescriptor(description='Weight for encoder 2 in fusion'))
        
        # Topic names
        self.declare_parameter('odom_topic', '/odom',
            ParameterDescriptor(description='Odometry topic name'))
        self.declare_parameter('imu_topic', '/imu/data',
            ParameterDescriptor(description='IMU topic name'))
        self.declare_parameter('target_frame', 'Assem1/odom',
            ParameterDescriptor(description='Target frame for TF lookup'))
        self.declare_parameter('source_frame', 'Assem1/base_link',
            ParameterDescriptor(description='Source frame for TF lookup'))
        self.declare_parameter('output_odom_topic', '/localization/odometry',
            ParameterDescriptor(description='Output odometry topic'))
        self.declare_parameter('output_pose_topic', '/localization/pose',
            ParameterDescriptor(description='Output pose topic'))
        
        # Publishing options
        self.declare_parameter('publish_tf', True,
            ParameterDescriptor(description='Publish TF transforms'))
        self.declare_parameter('publish_odom', True,
            ParameterDescriptor(description='Publish odometry messages'))
        self.declare_parameter('publish_pose', True,
            ParameterDescriptor(description='Publish pose messages'))
    
    def initialize_ekf(self):
        """Initialize Extended Kalman Filter with ROS2 parameters"""
        
        # Get parameters
        initial_cov = self.get_parameter('initial_covariance').get_parameter_value().double_array_value
        process_noise = self.get_parameter('process_noise').get_parameter_value().double_array_value
        odom_noise = self.get_parameter('odom_noise').get_parameter_value().double_array_value
        imu_noise = self.get_parameter('imu_noise').get_parameter_value().double_array_value
        
        # Initialize state and covariance
        self.state = np.zeros(6)  # [x, y, theta, vx, vy, omega]
        self.P = np.diag(initial_cov)
        self.Q = np.diag(process_noise)
        self.R_odom = np.diag(odom_noise)
        self.R_imu = np.diag(imu_noise)
        
        self.get_logger().info('EKF initialized with ROS2 parameters')
    
    def setup_qos_profiles(self):
        """Setup QoS profiles for different message types"""
        
        # Sensor data - best effort for real-time performance
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Output data - reliable for downstream nodes
        self.output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
    
    def setup_communication(self):
        """Setup publishers and subscribers"""
        
        # Get topic names from parameters
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        output_odom_topic = self.get_parameter('output_odom_topic').get_parameter_value().string_value
        output_pose_topic = self.get_parameter('output_pose_topic').get_parameter_value().string_value
        
        # Subscribers for standard ROS2 topics
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, self.sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, self.sensor_qos)

        # TF2 buffer and listener for transform data
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        if self.get_parameter('publish_odom').get_parameter_value().bool_value:
            self.odom_pub = self.create_publisher(
                Odometry, output_odom_topic, self.output_qos)
        
        if self.get_parameter('publish_pose').get_parameter_value().bool_value:
            self.pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, output_pose_topic, self.output_qos)
        
        # TF broadcaster
        if self.get_parameter('publish_tf').get_parameter_value().bool_value:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Communication setup complete')
    
    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.odom_data = msg
        self.get_logger().debug('Received odometry data', throttle_duration_sec=1.0)

    def get_tf_transform(self):
        """Get transform from TF tree"""
        try:
            # Get the latest transform from odom to base_link
            transform = self.tf_buffer.lookup_transform(
                'Assem1/odom',  # target frame (from your data)
                'Assem1/base_link',  # source frame (from your data)
                rclpy.time.Time(),  # get latest
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(f'TF lookup failed: {e}', throttle_duration_sec=1.0)
            return None
    
    def imu_callback(self, msg):
        """Callback for IMU data with EKF update"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if dt > 0 and dt < 1.0:  # Sanity check
            # Prediction step
            self.predict(dt)
            
            # Update with IMU
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
        """Update EKF with IMU data"""
        # Measurement model
        H = np.zeros((1, 6))
        H[0, 5] = 1  # omega
        
        # Innovation
        z = np.array([angular_velocity])
        y = z - H @ self.state
        
        # Update
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def update_callback(self):
        """Main update callback"""
        if self.odom_data is not None:
            # Fuse odometry and TF data
            x, y, theta = self.fuse_odom_and_tf()
        
            if x is not None:
                # Update EKF with fused data
                self.update_odometry(x, y, theta)
            
                # Publish results
                self.publish_results()
    
    def fuse_odom_and_tf(self):
        """Fuse odometry and TF data"""
        if self.odom_data is None:
            return None, None, None
    
        # Get data from odometry message
        odom_x = self.odom_data.pose.pose.position.x
        odom_y = self.odom_data.pose.pose.position.y
        odom_q = self.odom_data.pose.pose.orientation
        odom_euler = tf_transformations.euler_from_quaternion([odom_q.x, odom_q.y, odom_q.z, odom_q.w])
        odom_theta = odom_euler[2]
    
        # Get TF transform data
        tf_transform = self.get_tf_transform()
        if tf_transform is not None:
            tf_x = tf_transform.transform.translation.x
            tf_y = tf_transform.transform.translation.y
            tf_q = tf_transform.transform.rotation
            tf_euler = tf_transformations.euler_from_quaternion([tf_q.x, tf_q.y, tf_q.z, tf_q.w])
            tf_theta = tf_euler[2]
        
            # Fuse odometry and TF data (weighted average)
            weight_odom = 0.6  # Give more weight to odometry
            weight_tf = 0.4
        
            x_fused = weight_odom * odom_x + weight_tf * tf_x
            y_fused = weight_odom * odom_y + weight_tf * tf_y
        
            # Handle angle wrapping for theta fusion
            diff = tf_theta - odom_theta
            if diff > np.pi:
                tf_theta -= 2 * np.pi
            elif diff < -np.pi:
                tf_theta += 2 * np.pi
        
            theta_fused = weight_odom * odom_theta + weight_tf * tf_theta
        
            return x_fused, y_fused, theta_fused
        else:
            # Use only odometry data if TF is not available
            return odom_x, odom_y, odom_theta
    
    def update_odometry(self, x, y, theta):
        """Update EKF with odometry measurement"""
        H = np.zeros((3, 6))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # theta
        
        z = np.array([x, y, theta])
        y_k = z - H @ self.state
        y_k[2] = self.normalize_angle(y_k[2])
        
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y_k
        self.state[2] = self.normalize_angle(self.state[2])
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def publish_results(self):
        """Publish localization results"""
        current_time = self.get_clock().now().to_msg()
        
        # Publish odometry
        if hasattr(self, 'odom_pub'):
            odom_msg = self.create_odometry_message(current_time)
            self.odom_pub.publish(odom_msg)
        
        # Publish pose
        if hasattr(self, 'pose_pub'):
            pose_msg = self.create_pose_message(current_time)
            self.pose_pub.publish(pose_msg)
        
        # Publish transform
        if hasattr(self, 'tf_broadcaster'):
            tf_msg = self.create_transform_message(current_time)
            self.tf_broadcaster.sendTransform(tf_msg)
    
    def create_odometry_message(self, timestamp):
        """Create odometry message"""
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        msg.child_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        
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
        
        # Velocity
        msg.twist.twist.linear.x = self.state[3]
        msg.twist.twist.linear.y = self.state[4]
        msg.twist.twist.angular.z = self.state[5]
        
        # Covariance
        pose_cov = np.zeros(36)
        pose_cov[0] = self.P[0, 0]   # x
        pose_cov[7] = self.P[1, 1]   # y
        pose_cov[35] = self.P[2, 2]  # theta
        msg.pose.covariance = pose_cov.tolist()
        
        return msg
    
    def create_pose_message(self, timestamp):
        """Create pose with covariance message"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        
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
        pose_cov[0] = self.P[0, 0]   # x
        pose_cov[7] = self.P[1, 1]   # y
        pose_cov[35] = self.P[2, 2]  # theta
        msg.pose.covariance = pose_cov.tolist()
        
        return msg
    
    def create_transform_message(self, timestamp):
        """Create transform message"""
        msg = TransformStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        msg.child_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        
        # Translation
        msg.transform.translation.x = self.state[0]
        msg.transform.translation.y = self.state[1]
        msg.transform.translation.z = 0.0
        
        # Rotation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        
        return msg
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ROS2LocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
