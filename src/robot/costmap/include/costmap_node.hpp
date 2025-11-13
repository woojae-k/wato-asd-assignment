#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
  
    CostmapNode();

  private:

    //Subscription
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    //Publisher
    //Callback function
    void publishCostmap();

    double resolution_ = 0.1;      // meters/cell
    int map_width_ = 500;          // cells
    int map_height_ = 500;         // cells
    double origin_x_ = -25.0;      // meters (map_width_ / 2 * resolution_)
    double origin_y_ = -25.0;      // meters (map_height_ / 2 * resolution_)
    double inflation_radius_ = 1.0;  // meters

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    robot::CostmapCore costmap_;
};
 
#endif 