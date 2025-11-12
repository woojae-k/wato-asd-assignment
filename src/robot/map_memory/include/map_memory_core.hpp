#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace robot
{

/**
 * @class MapMemoryCore
 * @brief Handles the core logic for map memory, separate from ROS plumbing.
 * This class has no dependency on ROS message pointers in its API.
 */
class MapMemoryCore {
  public:
    /**
     * @brief Constructor for MapMemoryCore.
     * @param dist_thresh Minimum distance (meters) robot must travel to trigger an update.
     * @param map_w Width of the global map in meters.
     * @param map_h Height of the global map in meters.
     * @param map_res Resolution of the global map (meters/cell).
     * @param map_ox Origin X-coordinate of the global map (world frame).
     * @param map_oy Origin Y-coordinate of the global map (world frame).
     */
    explicit MapMemoryCore(double dist_thresh, double map_w, double map_h, double map_res, double map_ox, double map_oy);

    /**
     * @brief Processes the robot's current position.
     * Tracks robot's current position and checks if distance threshold is met.
     * @param x The current x-position of the robot.
     * @param y The current y-position of the robot.
     */
    void processOdometry(double x, double y, double z, double w);

    /**
     * @brief Processes an incoming costmap.
     * Stores the latest costmap.
     * @param msg The occupancy grid data.
     */
    void processCostmap(const nav_msgs::msg::OccupancyGrid& msg);

    /**
     * @brief Checks if conditions are met to update the map.
     * @return True if robot has moved required distance and a costmap is available.
     */
    bool shouldUpdate() const;

    /**
     * @brief Gets the current global map.
     * @return A const reference to the global map.
     */
    const nav_msgs::msg::OccupancyGrid& getGlobalMap() const;

    /**
     * @brief Resets the update flag after a successful map fusion.
     */
    void resetUpdateFlag();

    /**
     * @brief Fuses the latest costmap into the global map.
     * This performs the cell-by-cell update logic.
     */
    void integrateCostmap();

  private:
    // --- State Variables ---
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    double last_update_x_{0.0};
    double last_update_y_{0.0};
    double current_x_{0.0};
    double current_y_{0.0};
    double current_theta_{0.0};

    // --- Configuration ---
    // Member variables used in your constructor
    const double dist_thresh_;
    const double map_w_;
    const double map_h_;
    const double map_res_;
    const double map_ox_;
    const double map_oy_;

    // --- Flags ---
    bool costmap_received_{false};
    bool odom_received_{false};
    bool should_update_map_{false};
};

} // namespace robot

#endif // MAP_MEMORY_CORE_HPP_