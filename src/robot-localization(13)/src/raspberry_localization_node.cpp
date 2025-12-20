#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>

class RaspberryLocalizationNode : public rclcpp::Node
{
public:
    RaspberryLocalizationNode() : Node("raspberry_localization_node")
    {
        // Initialize EKF state: [x, y, theta, vx, vy, omega]
        state_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;  // Covariance matrix
        Q_ = Eigen::MatrixXd::Zero(6, 6);  // Process noise
        Q_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
        
        R_odom_ = Eigen::MatrixXd::Zero(3, 3);  // Wheel odometry noise
        R_odom_.diagonal() << 0.02, 0.02, 0.01;
        
        R_imu_ = Eigen::MatrixXd::Zero(1, 1);  // IMU noise
        R_imu_(0, 0) = 0.005;
        
        // QoS profiles
        auto sensor_qos = rclcpp::QoS(10).best_effort();
        auto reliable_qos = rclcpp::QoS(10).reliable();
        
        // Subscribers for Raspberry Pi topics
        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odom", sensor_qos,
            std::bind(&RaspberryLocalizationNode::wheelOdomCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", sensor_qos,
            std::bind(&RaspberryLocalizationNode::imuCallback, this, std::placeholders::_1));
        
        // Publishers
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/localization/odometry", reliable_qos);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", reliable_qos);
        
        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Timer for main fusion loop (50 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RaspberryLocalizationNode::fusionCallback, this));
        
        last_update_time_ = this->now();
        last_imu_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Raspberry Pi Localization Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  - /wheel_odom (nav_msgs/Odometry) - wheel encoder data");
        RCLCPP_INFO(this->get_logger(), "  - /imu_data (sensor_msgs/Imu) - IMU sensor data");
        RCLCPP_INFO(this->get_logger(), "Publishing to:");
        RCLCPP_INFO(this->get_logger(), "  - /localization/odometry (nav_msgs/Odometry)");
        RCLCPP_INFO(this->get_logger(), "  - /localization/pose (geometry_msgs/PoseWithCovarianceStamped)");
    }

private:
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_wheel_odom_ = msg;
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Wheel Odom: pos=(%.3f, %.3f), vel=(%.3f, %.3f)",
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->twist.twist.linear.x, msg->twist.twist.angular.z);
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - last_imu_time_).seconds();
        
        if (dt > 0 && dt < 1.0) {  // Sanity check
            predictWithImu(dt, msg);
            last_imu_time_ = current_time;
            
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "IMU: angular_vel=%.3f, linear_accel=(%.3f, %.3f)",
                msg->angular_velocity.z, msg->linear_acceleration.x, msg->linear_acceleration.y);
        }
    }
    
    void predictWithImu(double dt, const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        // State transition matrix
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3) = dt;  // x += vx * dt
        F(1, 4) = dt;  // y += vy * dt
        F(2, 5) = dt;  // theta += omega * dt
        
        // Use IMU angular velocity for better prediction
        double imu_omega = imu_msg->angular_velocity.z;
        
        // Predict state with IMU angular velocity
        Eigen::VectorXd predicted_state = F * state_;
        predicted_state(5) = imu_omega;  // Use IMU angular velocity directly
        predicted_state(2) = state_(2) + imu_omega * dt;  // Update theta with IMU
        predicted_state(2) = normalizeAngle(predicted_state(2));
        
        state_ = predicted_state;
        
        // Predict covariance
        P_ = F * P_ * F.transpose() + Q_;
        
        // Update with IMU angular velocity measurement
        updateImuAngularVelocity(imu_omega);
    }
    
    void updateImuAngularVelocity(double angular_velocity)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
        H(0, 5) = 1;  // omega measurement
        
        Eigen::VectorXd z(1);
        z(0) = angular_velocity;
        
        Eigen::VectorXd y = z - H * state_;
        
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        state_ = state_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    }
    
    void fusionCallback()
    {
        if (!latest_wheel_odom_) {
            return;
        }
        
        auto current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();
        
        if (dt > 0) {
            // Extract wheel odometry data
            double odom_x = latest_wheel_odom_->pose.pose.position.x;
            double odom_y = latest_wheel_odom_->pose.pose.position.y;
            
            // Convert quaternion to euler
            tf2::Quaternion q(
                latest_wheel_odom_->pose.pose.orientation.x,
                latest_wheel_odom_->pose.pose.orientation.y,
                latest_wheel_odom_->pose.pose.orientation.z,
                latest_wheel_odom_->pose.pose.orientation.w);
            
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            double odom_theta = yaw;
            
            // Extract velocity data
            double odom_vx = latest_wheel_odom_->twist.twist.linear.x;
            double odom_vy = latest_wheel_odom_->twist.twist.linear.y;
            double odom_omega = latest_wheel_odom_->twist.twist.angular.z;
            
            // Update EKF with wheel odometry measurement
            updateWheelOdometry(odom_x, odom_y, odom_theta, odom_vx, odom_vy, odom_omega);
            
            // Publish fused results
            publishFusedOdometry();
            publishPose();
            publishTransform();
            
            last_update_time_ = current_time;
        }
    }
    
    void updateWheelOdometry(double x, double y, double theta, double vx, double vy, double omega)
    {
        // Measurement model for full state observation
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);
        
        Eigen::VectorXd z(6);
        z << x, y, theta, vx, vy, omega;
        
        Eigen::VectorXd y_k = z - H * state_;
        y_k(2) = normalizeAngle(y_k(2));  // Handle angle wrapping
        
        // Use full covariance for odometry + velocity
        Eigen::MatrixXd R_full = Eigen::MatrixXd::Zero(6, 6);
        R_full.block<3, 3>(0, 0) = R_odom_;  // Position and orientation
        R_full.block<3, 3>(3, 3) = Eigen::Vector3d(0.05, 0.05, 0.02).asDiagonal();  // Velocity measurements
        
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_full;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        state_ = state_ + K * y_k;
        state_(2) = normalizeAngle(state_(2));
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "EKF State: pos=(%.3f, %.3f), theta=%.1f°, vel=(%.3f, %.3f)",
            state_(0), state_(1), state_(2) * 180.0 / M_PI, state_(3), state_(5));
    }
    
    void publishFusedOdometry()
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        
        // Position from EKF state
        msg.pose.pose.position.x = state_(0);
        msg.pose.pose.position.y = state_(1);
        msg.pose.pose.position.z = 0.0;
        
        // Orientation from EKF state
        tf2::Quaternion q;
        q.setRPY(0, 0, state_(2));
        msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Velocity from EKF state
        msg.twist.twist.linear.x = state_(3);
        msg.twist.twist.linear.y = state_(4);
        msg.twist.twist.angular.z = state_(5);
        
        // Covariance from EKF
        std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
        msg.pose.covariance[0] = P_(0, 0);   // x variance
        msg.pose.covariance[7] = P_(1, 1);   // y variance
        msg.pose.covariance[35] = P_(2, 2);  // theta variance
        
        std::fill(msg.twist.covariance.begin(), msg.twist.covariance.end(), 0.0);
        msg.twist.covariance[0] = P_(3, 3);   // vx variance
        msg.twist.covariance[7] = P_(4, 4);   // vy variance
        msg.twist.covariance[35] = P_(5, 5);  // omega variance
        
        fused_odom_pub_->publish(msg);
    }
    
    void publishPose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        
        // Position
        msg.pose.pose.position.x = state_(0);
        msg.pose.pose.position.y = state_(1);
        msg.pose.pose.position.z = 0.0;
        
        // Orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, state_(2));
        msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Covariance
        std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
        msg.pose.covariance[0] = P_(0, 0);
        msg.pose.covariance[7] = P_(1, 1);
        msg.pose.covariance[35] = P_(2, 2);
        
        pose_pub_->publish(msg);
    }
    
    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        
        // Translation
        t.transform.translation.x = state_(0);
        t.transform.translation.y = state_(1);
        t.transform.translation.z = 0.0;
        
        // Rotation
        tf2::Quaternion q;
        q.setRPY(0, 0, state_(2));
        t.transform.rotation = tf2::toMsg(q);
        
        tf_broadcaster_->sendTransform(t);
    }
    
    double normalizeAngle(double angle)
    {
        return atan2(sin(angle), cos(angle));
    }
    
    // Member variables
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_, Q_, R_odom_, R_imu_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry::SharedPtr latest_wheel_odom_;
    rclcpp::Time last_update_time_, last_imu_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RaspberryLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
