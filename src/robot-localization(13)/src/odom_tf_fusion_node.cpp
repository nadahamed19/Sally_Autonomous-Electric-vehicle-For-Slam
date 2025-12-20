#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>

class OdomTFFusionNode : public rclcpp::Node
{
public:
    OdomTFFusionNode() : Node("odom_tf_fusion_node")
    {
        // Initialize EKF state: [x, y, theta, vx, vy, omega]
        state_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
        
        R_odom_ = Eigen::MatrixXd::Zero(3, 3);
        R_odom_.diagonal() << 0.02, 0.02, 0.01;
        
        R_tf_ = Eigen::MatrixXd::Zero(3, 3);
        R_tf_.diagonal() << 0.03, 0.03, 0.015;
        
        // QoS profiles
        auto sensor_qos = rclcpp::QoS(10).best_effort();
        auto reliable_qos = rclcpp::QoS(10).reliable();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", sensor_qos,
            std::bind(&OdomTFFusionNode::odomCallback, this, std::placeholders::_1));
        
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", sensor_qos,
            std::bind(&OdomTFFusionNode::tfCallback, this, std::placeholders::_1));
        
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
            std::bind(&OdomTFFusionNode::fusionCallback, this));
        
        last_update_time_ = this->now();
        
        // Fusion weights
        odom_weight_ = 0.7;
        tf_weight_ = 0.3;
        
        RCLCPP_INFO(this->get_logger(), "Odometry-TF Fusion Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /odom, /tf");
        RCLCPP_INFO(this->get_logger(), "Fusion weights: odom=%.1f, tf=%.1f", odom_weight_, tf_weight_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = msg;
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Odom: pos=(%.3f, %.3f), vel=(%.3f, %.3f)",
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->twist.twist.linear.x, msg->twist.twist.angular.z);
    }
    
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            if (transform.header.frame_id == "Assem1/odom" && 
                transform.child_frame_id == "Assem1/base_link") {
                
                latest_tf_transform_ = std::make_shared<geometry_msgs::msg::TransformStamped>(transform);
                last_tf_time_ = this->now();
                
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "TF: pos=(%.3f, %.3f), frame=%s -> %s",
                    transform.transform.translation.x, transform.transform.translation.y,
                    transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
                break;
            }
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
    
    void fusionCallback()
    {
        if (!latest_odom_) return;
        
        auto current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();
        
        if (dt > 0) {
            predict(dt);
            
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
            
            double odom_vx = latest_odom_->twist.twist.linear.x;
            double odom_vy = latest_odom_->twist.twist.linear.y;
            double odom_omega = latest_odom_->twist.twist.angular.z;
            
            // Update with odometry
            updateOdometry(odom_x, odom_y, odom_theta, odom_vx, odom_vy, odom_omega);
            
            // Update with TF if available and recent
            if (latest_tf_transform_ && last_tf_time_.has_value()) {
                double tf_age = (current_time - last_tf_time_.value()).seconds();
                if (tf_age < 0.5) {  // Only use TF data if it's less than 500ms old
                    updateWithTF();
                }
            }
            
            publishResults();
            last_update_time_ = current_time;
        }
    }
    
    void updateOdometry(double x, double y, double theta, double vx, double vy, double omega)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);
        
        Eigen::VectorXd z(6);
        z << x, y, theta, vx, vy, omega;
        
        Eigen::VectorXd y_k = z - H * state_;
        y_k(2) = normalizeAngle(y_k(2));
        
        Eigen::MatrixXd R_full = Eigen::MatrixXd::Zero(6, 6);
        R_full.block<3, 3>(0, 0) = R_odom_;
        R_full.block<3, 3>(3, 3) = Eigen::Vector3d(0.05, 0.05, 0.02).asDiagonal();
        
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_full;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        state_ = state_ + K * y_k;
        state_(2) = normalizeAngle(state_(2));
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    }
    
    void updateWithTF()
    {
        if (!latest_tf_transform_) return;
        
        double tf_x = latest_tf_transform_->transform.translation.x;
        double tf_y = latest_tf_transform_->transform.translation.y;
        
        tf2::Quaternion tf_q(
            latest_tf_transform_->transform.rotation.x,
            latest_tf_transform_->transform.rotation.y,
            latest_tf_transform_->transform.rotation.z,
            latest_tf_transform_->transform.rotation.w);
        
        double tf_roll, tf_pitch, tf_yaw;
        tf2::Matrix3x3(tf_q).getRPY(tf_roll, tf_pitch, tf_yaw);
        
        Eigen::MatrixXd H_tf = Eigen::MatrixXd::Zero(3, 6);
        H_tf(0, 0) = 1;
        H_tf(1, 1) = 1;
        H_tf(2, 2) = 1;
        
        Eigen::VectorXd z_tf(3);
        z_tf << tf_x, tf_y, tf_yaw;
        
        Eigen::VectorXd y_k_tf = z_tf - H_tf * state_;
        y_k_tf(2) = normalizeAngle(y_k_tf(2));
        
        Eigen::MatrixXd S_tf = H_tf * P_ * H_tf.transpose() + R_tf_;
        Eigen::MatrixXd K_tf = P_ * H_tf.transpose() * S_tf.inverse();
        
        state_ = state_ + K_tf * y_k_tf;
        state_(2) = normalizeAngle(state_(2));
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K_tf * H_tf) * P_;
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
    Eigen::MatrixXd P_, Q_, R_odom_, R_tf_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> latest_tf_transform_;
    rclcpp::Time last_update_time_;
    std::optional<rclcpp::Time> last_tf_time_;
    
    double odom_weight_, tf_weight_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTFFusionNode>());
    rclcpp::shutdown();
    return 0;
}
