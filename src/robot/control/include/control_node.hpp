#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

/**
 * @class ControlNode
 * @brief The ROS 2 wrapper for the ControlCore.
 * This class handles all ROS 2 subscriptions, publications, and timers.
 * It passes data to and from the ControlCore, which contains the algorithm.
 */
class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    // --- ROS Callbacks ---
    /**
     * @brief Receives the path and passes it to the controller.
     */
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    /**
     * @brief Receives odometry and passes it to the controller.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief The main control loop, triggered by a timer.
     * It requests a velocity command from the core and publishes it.
     */
    void timerCallback();

    // --- ROS Interface ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // --- Core Logic ---
    robot::ControlCore controller_;
};

#endif // CONTROL_NODE_HPP_