#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

/**
 * @class CostmapNode
 * @brief ROS 2 Node for generating a costmap from LaserScan data.
 *
 * Subscribes to /lidar (LaserScan) and publishes to /costmap (OccupancyGrid).
 * Uses CostmapCore for the underlying grid logic.
 */
class CostmapNode : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor
     * @param options Node options for initialization
     */
    explicit CostmapNode(const rclcpp::NodeOptions & options);

  private:
    /**
     * @brief Callback function for the LaserScan subscriber.
     * @param scan The received LaserScan message
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Populates and publishes the OccupancyGrid message.
     */
    void publishCostmap();

    // --- Parameters (hardcoded as per example) ---
    double resolution_ = 0.1;      // meters/cell
    int map_width_ = 200;          // cells
    int map_height_ = 200;         // cells
    double origin_x_ = -10.0;      // meters (map_width_ / 2 * resolution_)
    double origin_y_ = -10.0;      // meters (map_height_ / 2 * resolution_)
    double inflation_radius_ = 1.0;  // meters

    // --- Message Members ---
    nav_msgs::msg::OccupancyGrid costmap_msg_;
    
};

#endif  // COSTMAP_NODE_HPP_