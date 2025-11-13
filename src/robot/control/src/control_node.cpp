#include "control_node.hpp"

ControlNode::ControlNode() 
: Node("control_node"), // Node name from your instructions
  controller_(this->get_logger())
{
    RCLCPP_INFO(this->get_logger(), "ControlNode starting up...");

    // QoS settings
    // Path should be reliable
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    // Odom is high-frequency, best-effort is fine
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // Subscribers (as in pseudo-code)
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", reliable_qos, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", best_effort_qos, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    // Publisher (as in pseudo-code)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer (100ms / 10 Hz, as in pseudo-code)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::timerCallback, this));
        
    RCLCPP_INFO(this->get_logger(), "ControlNode initialized successfully.");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    // Pass path to the core
    controller_.setPath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Pass odometry to the core
    controller_.setOdometry(*msg);
}

void ControlNode::timerCallback() {
    // 1. Compute velocity
    auto cmd_vel_opt = controller_.computeVelocityCommand();

    // 2. Publish command
    if (cmd_vel_opt.has_value()) {
        // If we get a command (either for motion or to stop), publish it
        cmd_vel_pub_->publish(cmd_vel_opt.value());
    }
    // If std::nullopt, we are just waiting for data, so we don't publish anything
    // and let the robot continue its previous command (which should be zero
    // if the path was cleared).
    // A more robust system might publish zero velocity if no command
    // is received for a certain duration.
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}