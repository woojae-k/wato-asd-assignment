#include "control_core.hpp"
#include <algorithm> // For std::min/max
#include <cmath>     // For M_PI

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger)
{
    // Initialize parameters as specified in the pseudo-code
    lookahead_distance_ = 1.0; // Lookahead distance
    goal_tolerance_ = 0.1;     // Distance to consider the goal reached
    linear_speed_ = 0.5;       // Constant forward speed

    RCLCPP_INFO(logger_, "ControlCore initialized with lookahead=%.2f, speed=%.2f, goal_tolerance=%.2f",
        lookahead_distance_, linear_speed_, goal_tolerance_);
}

void ControlCore::setPath(const nav_msgs::msg::Path& path) {
    if (path.poses.empty()) {
        RCLCPP_WARN(logger_, "Received an empty path. Clearing current path.");
        current_path_ = std::nullopt;
    } else {
        current_path_ = path;
        RCLCPP_INFO(logger_, "New path with %zu poses received.", path.poses.size());
    }
}

void ControlCore::setOdometry(const nav_msgs::msg::Odometry& odom) {
    robot_odom_ = odom;
}

double ControlCore::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const {
    // Standard ROS 2 (Z-Y-X) quaternion to yaw conversion
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double ControlCore::distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

int ControlCore::findClosestWaypoint() const {
    int closest_idx = -1;
    double min_dist = std::numeric_limits<double>::max();
    const auto& robot_pos = robot_odom_->pose.pose.position;

    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double d = distance(robot_pos, current_path_->poses[i].pose.position);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }
    return closest_idx;
}

std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(int start_index) const {
    const auto& robot_pos = robot_odom_->pose.pose.position;

    // Iterate along the path from the closest waypoint
    for (size_t i = start_index; i < current_path_->poses.size() - 1; ++i) {
        const auto& p1 = current_path_->poses[i].pose.position;
        const auto& p2 = current_path_->poses[i + 1].pose.position;

        // Find intersection of a line segment (p1, p2) and a circle (robot_pos, lookahead_distance_)
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double f_x = p1.x - robot_pos.x;
        double f_y = p1.y - robot_pos.y;

        double a = dx*dx + dy*dy;
        double b = 2.0 * (f_x*dx + f_y*dy);
        double c = f_x*f_x + f_y*f_y - lookahead_distance_*lookahead_distance_;
        double discriminant = b*b - 4.0*a*c;

        if (discriminant >= 0) {
            discriminant = std::sqrt(discriminant);
            double t1 = (-b - discriminant) / (2.0 * a);
            double t2 = (-b + discriminant) / (2.0 * a);

            // Prioritize t2 as it's further along the path
            if (t2 >= 0 && t2 <= 1.0) {
                geometry_msgs::msg::Point p;
                p.x = p1.x + t2 * dx;
                p.y = p1.y + t2 * dy;
                return p;
            }
            if (t1 >= 0 && t1 <= 1.0) {
                geometry_msgs::msg::Point p;
                p.x = p1.x + t1 * dx;
                p.y = p1.y + t1 * dy;
                return p;
            }
        }
    }

    // --- START FIX: Robust Fallback Logic ---
    // No intersection found on segments.
    
    // Fallback 1: Check the last point (goal). If it's within reach, target it.
    const auto& last_point = current_path_->poses.back().pose.position;
    if (distance(robot_pos, last_point) <= lookahead_distance_) {
        return last_point;
    }

    // Fallback 2: No intersection and goal is far away.
    // This means the path is probably behind or far to the side.
    // Just return the *closest* point on the path as the target.
    // This will force the robot to turn towards the path.
    return current_path_->poses[start_index].pose.position;
    // --- END FIX ---
}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeVelocityCommand() {
    // --- 1. Sanity Checks (as in pseudo-code) ---
    if (!current_path_ || !robot_odom_) {
        return std::nullopt; // No path or odom, do nothing
    }

    if (current_path_->poses.empty()) {
        RCLCPP_WARN(logger_, "Path is empty, cannot compute command.");
        return geometry_msgs::msg::Twist(); // Return zero velocity
    }

    const auto& robot_pos = robot_odom_->pose.pose.position;
    const auto& goal_pos = current_path_->poses.back().pose.position;

    // --- 2. Check if Goal Reached ---
    if (distance(robot_pos, goal_pos) < goal_tolerance_) {
        RCLCPP_INFO_ONCE(logger_, "Goal reached!");
        current_path_ = std::nullopt; // Clear the path
        return geometry_msgs::msg::Twist(); // Return zero velocity
    }

    // --- 3. Find Lookahead Point ---
    int closest_idx = findClosestWaypoint();
    // Thanks to our fix, opt_lookahead_pt will now *always* have a value
    // if the path is not empty.
    auto opt_lookahead_pt = findLookaheadPoint(closest_idx);
    
    auto lookahead_pt = opt_lookahead_pt.value();

    // --- 4. Compute Steering Angle (alpha) ---
    double robot_yaw = getYawFromQuaternion(robot_odom_->pose.pose.orientation);
    double angle_to_goal = std::atan2(lookahead_pt.y - robot_pos.y,
                                      lookahead_pt.x - robot_pos.x);
    double alpha = angle_to_goal - robot_yaw;

    // Normalize alpha to [-pi, pi]
    alpha = std::atan2(std::sin(alpha), std::cos(alpha));

    // --- 5. Compute Velocity Command (with FIX) ---
    geometry_msgs::msg::Twist cmd_vel;
    
    // --- START FIX: Handle "behind" target ---
    // Check if the target is "behind" the robot (angle > 90 degrees)
    if (std::abs(alpha) > (M_PI / 2.0)) {
        // Target is behind: stop and turn
        cmd_vel.linear.x = 0.0;

        // Use a simple proportional control for turning, with a max speed
        double Kp_turn = 1.0;
        double max_rot_speed = 0.7; // rad/s (approx 40 deg/s)
        double rot_vel = Kp_turn * alpha;
        
        // Clamp the rotation velocity
        cmd_vel.angular.z = std::max(-max_rot_speed, std::min(max_rot_speed, rot_vel));

    } else {
        // Target is in front: use standard pure pursuit
        // We use the 'lookahead_distance_' parameter in the denominator
        // for stability, preventing division by a small 'L'.
        double omega = (2.0 * linear_speed_ * std::sin(alpha)) / lookahead_distance_;
        
        cmd_vel.linear.x = linear_speed_;
        cmd_vel.angular.z = omega;
    }
    // --- END FIX ---

    return cmd_vel;
}

} // namespace robot