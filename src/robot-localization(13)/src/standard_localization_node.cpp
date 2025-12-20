#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

class StandardLocalizationNode : public rclcpp::Node
{
public:
    StandardLocalizationNode() : Node("standard_localization_node")
    {
        // Initialize EKF
        state_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
        
        R_odom_ = Eigen::MatrixXd::Zero(3, 3);
        R_odom_.diagonal() << 0.05, 0.05, 0.02;
        
        R_imu_ = Eigen::MatrixXd::Zero(1, 1);
        R_imu_(0, 0) = 0.01;
        
        auto sensor_qos = rclcpp::QoS(10).best_effort();
        auto reliable_qos = rclcpp::QoS(10).reliable();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", sensor_qos,
            std::bind(&StandardLocalizationNode::odomCallback, this, std::placeholders::_1));
        
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", sensor_qos,
            std::bind(&StandardLocalizationNode::tfCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos,
            std::bind(&StandardLocalizationNode::imuCallback, this, std::placeholders::_1));
        
        // Publishers
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/localization/odometry", reliable_qos);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", reliable_qos);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&StandardLocalizationNode::fusionCallback, this));
        
        last_update_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Standard Localization Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /odom, /tf, /imu/data");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = msg;
    }
    
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            if (transform.header.frame_id == "Assem1/odom" && 
                transform.child_frame_id == "Assem1/base_link") {
                latest_tf_transform_ = std::make_shared<geometry_msgs::msg::TransformStamped>(transform);
                break;
            }
        }
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();
        
        if (dt > 0 && dt < 1.0) {
            predict(dt);
            updateImu(msg->angular_velocity.z);
            last_update_time_ = current_time;
        }
    }
    
    void predict(double dt)
    {
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
    
    void fusionCallback()
    {
        if (!latest_odom_) return;
        
        // Extract odometry data
        double odom_x = latest_odom_->pose.pose.position.x;
        double odom_y = latest_odom_->pose.pose.position.y;
        
        tf2::Quaternion odom_q(
            latest_odom_->pose.pose.orientation.x,
            latest_odom_->pose.pose.orientation.y,
            latest_odom_->pose.pose.orientation.z,
            latest_odom_->pose.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(odom_q).getRPY(roll, pitch, yaw);
        double odom_theta = yaw;
        
        // Fuse with TF if available
        double x_fused = odom_x;
        double y_fused = odom_y;
        double theta_fused = odom_theta;
        
        if (latest_tf_transform_) {
            double tf_x = latest_tf_transform_->transform.translation.x;
            double tf_y = latest_tf_transform_->transform.translation.y;
            
            tf2::Quaternion tf_q(
                latest_tf_transform_->transform.rotation.x,
                latest_tf_transform_->transform.rotation.y,
                latest_tf_transform_->transform.rotation.z,
                latest_tf_transform_->transform.rotation.w);
            
            double tf_roll, tf_pitch, tf_yaw;
            tf2::Matrix3x3(tf_q).getRPY(tf_roll, tf_pitch, tf_yaw);
            
            // Weighted fusion
            double w_odom = 0.7;
            double w_tf = 0.3;
            
            x_fused = w_odom * odom_x + w_tf * tf_x;
            y_fused = w_odom * odom_y + w_tf * tf_y;
            
            // Handle angle wrapping
            double diff = tf_yaw - odom_theta;
            if (diff > M_PI) tf_yaw -= 2 * M_PI;
            else if (diff < -M_PI) tf_yaw += 2 * M_PI;
            
            theta_fused = w_odom * odom_theta + w_tf * tf_yaw;
        }
        
        updatePosition(x_fused, y_fused, theta_fused);
        publishResults();
    }
    
    void updatePosition(double x, double y, double theta)
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
        publishFusedOdometry();
        publishPose();
        publishTransform();
    }
    
    void publishFusedOdometry()
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
        
        fused_odom_pub_->publish(msg);
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
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> latest_tf_transform_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandardLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
