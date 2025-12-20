#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sstream>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // Set your Arduino serial port
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened.");
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ControlNode::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    int serial_fd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        int left_speed = static_cast<int>((linear - angular) * 100);
        int right_speed = static_cast<int>((linear + angular) * 100);

        std::stringstream ss;
        ss << "M:" << left_speed << ":" << right_speed << "\n";
        std::string command = ss.str();

        write(serial_fd_, command.c_str(), command.size());
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
