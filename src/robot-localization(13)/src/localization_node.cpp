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

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {
        // Initialize EKF state: [x, y, theta, vx, vy, omega]
        state_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
        
        R_odom_ = Eigen::MatrixXd::Zero(3, 3);
        R_odom_.diagonal() << 0.05, 0.05, 0.02;
        
        R_imu_ = Eigen::MatrixXd::Zero(1, 1);
        R_imu_(0, 0) = 0.01;
        
        // QoS profiles
        auto sensor_qos = rclcpp::QoS(10).best_effort();
        auto reliable_qos = rclcpp::QoS(10).reliable();
        
        // Subscribers for dual encoder setup
        encoder1_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/encoder1/odometry", sensor_qos,
            std::bind(&LocalizationNode::encoder1Callback, this, std::placeholders::_1));
        
        encoder2_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/encoder2/odometry", sensor_qos,
            std::bind(&LocalizationNode::encoder2Callback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos,
            std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));
        
        // Publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/localization/odometry", reliable_qos);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", reliable_qos);
        
        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Timer for main fusion loop (50 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LocalizationNode::updateLocalization, this));
        
        last_update_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Dual Encoder Localization Node initialized");
    }

private:
    void encoder1Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        encoder1_data_ = msg;
        RCLCPP_DEBUG(this->get_logger(), "Received encoder1 data");
    }
    
    void encoder2Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        encoder2_data_ = msg;
        RCLCPP_DEBUG(this->get_logger(), "Received encoder2 data");
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();
        
        if (dt > 0) {
            predict(dt);
            updateImu(msg->angular_velocity.z);
            last_update_time_ = current_time;
        }
    }
    
    void predict(double dt)
    {
        if (dt <= 0) return;
        
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;
        
        state_ = F * state_;
        state_(2) = normalizeAngle(state_(2));
        P_ = F * P_ * F.transpose() + Q_;
    }
    
    void updateImu(double angular_velocity)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
        H(0, 5) = 1;
        
        Eigen::VectorXd z(1);
        z(0) = angular_velocity;
        
        Eigen::VectorXd y = z - H * state_;
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        state_ = state_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    }
    
    void updateLocalization()
    {
        auto fused_data = fuseEncoderData();
        if (fused_data.has_value()) {
            auto [x, y, theta] = fused_data.value();
            updateOdometry(x, y, theta);
            publishResults();
        }
    }
    
    std::optional<std::tuple<double, double, double>> fuseEncoderData()
    {
        if (!encoder1_data_ || !encoder2_data_) {
            return std::nullopt;
        }
        
        // Simple fusion: average the positions and orientations
        double x1 = encoder1_data_->pose.pose.position.x;
        double y1 = encoder1_data_->pose.pose.position.y;
        double x2 = encoder2_data_->pose.pose.position.x;
        double y2 = encoder2_data_->pose.pose.position.y;
        
        double x_fused = (x1 + x2) / 2.0;
        double y_fused = (y1 + y2) / 2.0;
        
        // Convert quaternions to euler and average
        tf2::Quaternion q1(
            encoder1_data_->pose.pose.orientation.x,
            encoder1_data_->pose.pose.orientation.y,
            encoder1_data_->pose.pose.orientation.z,
            encoder1_data_->pose.pose.orientation.w);
        
        tf2::Quaternion q2(
            encoder2_data_->pose.pose.orientation.x,
            encoder2_data_->pose.pose.orientation.y,
            encoder2_data_->pose.pose.orientation.z,
            encoder2_data_->pose.pose.orientation.w);
        
        double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
        tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
        tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);
        
        // Handle angle wrapping
        double diff = yaw2 - yaw1;
        if (diff > M_PI) yaw2 -= 2 * M_PI;
        else if (diff < -M_PI) yaw2 += 2 * M_PI;
        
        double theta_fused = (yaw1 + yaw2) / 2.0;
        
        return std::make_tuple(x_fused, y_fused, theta_fused);
    }
    
    void updateOdometry(double x, double y, double theta)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H(0, 0) = 1;
        H(1, 1) = 1;
        H(2, 2) = 1;
        
        Eigen::VectorXd z(3);
        z << x, y, theta;
        
        Eigen::VectorXd y_k = z - H * state_;
        y_k(2) = normalizeAngle(y_k(2));
        
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        state_ = state_ + K * y_k;
        state_(2) = normalizeAngle(state_(2));
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    }
    
    void publishResults()
    {
        publishOdometry();
        publishPose();
        publishTransform();
    }
    
    void publishOdometry()
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        
        msg.pose.pose.position.x = state_(0);
        msg.pose.pose.position.y = state_(1);
        msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, state_(2));
        msg.pose.pose.orientation = tf2::toMsg(q);
        
        msg.twist.twist.linear.x = state_(3);
        msg.twist.twist.linear.y = state_(4);
        msg.twist.twist.angular.z = state_(5);
        
        odom_pub_->publish(msg);
    }
    
    void publishPose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        
        msg.pose.pose.position.x = state_(0);
        msg.pose.pose.position.y = state_(1);
        msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, state_(2));
        msg.pose.pose.orientation = tf2::toMsg(q);
        
        pose_pub_->publish(msg);
    }
    
    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        
        t.transform.translation.x = state_(0);
        t.transform.translation.y = state_(1);
        t.transform.translation.z = 0.0;
        
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
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder1_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry::SharedPtr encoder1_data_;
    nav_msgs::msg::Odometry::SharedPtr encoder2_data_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
