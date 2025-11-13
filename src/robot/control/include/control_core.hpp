#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>
#include <optional>
#include <vector>

namespace robot
{

/**
 * @class ControlCore
 * @brief Implements the core logic for the Pure Pursuit path tracking algorithm.
 * This class is decoupled from ROS plumbing (subscribers, timers, etc.)
 * and contains only the state and calculations needed for the algorithm.
 */
class ControlCore {
  public:
    /**
     * @brief Constructor for ControlCore.
     * @param logger A ROS 2 logger for logging internal state.
     */
    explicit ControlCore(const rclcpp::Logger& logger);

    /**
     * @brief Sets the path for the controller to follow.
     * @param path The new path from the /path topic.
     */
    void setPath(const nav_msgs::msg::Path& path);

    /**
     * @brief Updates the robot's current position and orientation.
     * @param odom The new odometry message from /odom/filtered.
     */
    void setOdometry(const nav_msgs::msg::Odometry& odom);

    /**
     * @brief Main computation loop for the controller.
     * Calculates the required Twist command based on current path and odometry.
     * @return A Twist message if control is active.
     * std::nullopt if waiting for data.
     * A zero-velocity Twist if the goal is reached or an error occurs.
     */
    std::optional<geometry_msgs::msg::Twist> computeVelocityCommand();

  private:
    /**
     * @brief Finds the waypoint on the path closest to the robot.
     * @return Index of the closest waypoint in the path.
     */
    int findClosestWaypoint() const;

    /**
     * @brief Finds the target lookahead point on the path based on lookahead_distance_.
     * This function searches forward from a given start index and finds
     * the intersection of the path with a circle around the robot.
     * @param start_index The path index from which to start searching.
     * @return The 2D lookahead point, or std::nullopt if none is found.
     */
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(int start_index) const;

    /**
     * @brief Extracts the 2D yaw angle (rotation around Z) from a 3D quaternion.
     * @param quat The quaternion to convert.
     * @return The yaw angle in radians.
     */
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) const;

    /**
     * @brief Calculates the 2D Euclidean distance between two points.
     */
    double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;

    // --- Member Variables ---
    rclcpp::Logger logger_;
    
    // Stored data
    std::optional<nav_msgs::msg::Path> current_path_;
    std::optional<nav_msgs::msg::Odometry> robot_odom_;

    // Parameters from the pseudo-code example
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

} // namespace robot

#endif // CONTROL_CORE_HPP_