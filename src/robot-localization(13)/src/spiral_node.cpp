#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class SpiralNode : public rclcpp::Node
{
public:
    SpiralNode() : Node("spiral_node")
    {
        // Subscribe to localization pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", 10,
            std::bind(&SpiralNode::poseCallback, this, std::placeholders::_1));
        
        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Timer for spiral control
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&SpiralNode::spiralControl, this));
        
        // Spiral parameters
        spiral_center_x_ = 0.0;
        spiral_center_y_ = 0.0;
        spiral_radius_ = 0.1;  // Starting radius
        spiral_growth_rate_ = 0.01;  // How fast spiral grows
        angular_velocity_ = 0.5;  // rad/s
        
        start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Spiral Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /localization/pose");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /cmd_vel");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Extract current robot pose
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract orientation (convert quaternion to yaw)
        auto q = msg->pose.pose.orientation;
        current_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        // Check pose uncertainty (optional)
        double pos_uncertainty = sqrt(msg->pose.covariance[0] + msg->pose.covariance[7]);
        if (pos_uncertainty > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "High position uncertainty: %.3f m", pos_uncertainty);
        }
        
        pose_received_ = true;
    }
    
    void spiralControl()
    {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "No pose data received yet");
            return;
        }
        
        // Calculate time since start
        double elapsed_time = (this->now() - start_time_).seconds();
        
        // Calculate desired spiral position
        double current_radius = spiral_radius_ + spiral_growth_rate_ * elapsed_time;
        double angle = angular_velocity_ * elapsed_time;
        
        double target_x = spiral_center_x_ + current_radius * cos(angle);
        double target_y = spiral_center_y_ + current_radius * sin(angle);
        
        // Calculate errors
        double error_x = target_x - current_x_;
        double error_y = target_y - current_y_;
        double distance_error = sqrt(error_x * error_x + error_y * error_y);
        
        // Calculate desired heading
        double desired_yaw = atan2(error_y, error_x);
        double yaw_error = normalizeAngle(desired_yaw - current_yaw_);
        
        // Simple proportional control
        auto cmd_msg = geometry_msgs::msg::Twist();
        
        // Linear velocity (proportional to distance error)
        double kp_linear = 1.0;
        cmd_msg.linear.x = std::min(kp_linear * distance_error, 0.5); // Max 0.5 m/s
        
        // Angular velocity (proportional to yaw error)
        double kp_angular = 2.0;
        cmd_msg.angular.z = kp_angular * yaw_error;
        
        // Limit angular velocity
        cmd_msg.angular.z = std::max(-1.0, std::min(1.0, cmd_msg.angular.z));
        
        cmd_vel_pub_->publish(cmd_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Spiral: target=(%.3f,%.3f), current=(%.3f,%.3f), error=%.3f",
            target_x, target_y, current_x_, current_y_, distance_error);
    }
    
    double normalizeAngle(double angle)
    {
        return atan2(sin(angle), cos(angle));
    }
    
    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Current robot state
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    bool pose_received_ = false;
    
    // Spiral parameters
    double spiral_center_x_;
    double spiral_center_y_;
    double spiral_radius_;
    double spiral_growth_rate_;
    double angular_velocity_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpiralNode>());
    rclcpp::shutdown();
    return 0;
}
