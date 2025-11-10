#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_memory_core.hpp"
#include <memory>

/**
 * @class MapMemoryNode
 * @brief The ROS 2 Node wrapper for the MapMemoryCore.
 */
class MapMemoryNode : public rclcpp::Node {
  public:
    /**
     * @brief Constructor for the MapMemoryNode.
     */
    MapMemoryNode();

  private:
    /**
     * @brief Timer callback, triggered periodically to check for map updates.
     */
    void timerCallback();

    /**
     * @brief ROS Subscriber callback for odometry.
     * @param msg The received Odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief ROS Subscriber callback for the local costmap.
     * @param msg The received OccupancyGrid message.
     */
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // ROS 2 constructs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    const double distance_threshold_ = 1.5;    
    const double map_width_meters_ = 30.0;  
    const double map_height_meters_ = 30.0;  
    const double map_resolution_ = 0.05;   
    const double map_origin_x_ = -15.0;  
    const double map_origin_y_ = -15.0; 

    const double update_hz = 1.0;
    const double dist_thresh = 1.5; // For logging only
    
    // The core logic implementation
    robot::MapMemoryCore mapmemory_;
};

#endif