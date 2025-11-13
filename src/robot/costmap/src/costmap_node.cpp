#include <chrono>
#include <memory>
#include "costmap_node.hpp"


// Initialize the constructs and their parameters
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(resolution_, map_width_, map_height_, origin_x_, origin_y_, inflation_radius_)) {
  // Initialize publihser
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10);

  //Initialize subscriber
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "CostmapNode initialized.");

  
}
 
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", scan->ranges.size());

  costmap_.initialize();

  costmap_.processLidar(scan);

  costmap_.inflateObstacles();
  publishCostmap();  
}

void CostmapNode::publishCostmap() {

  auto costmap_msg = costmap_.getCostmap();
  costmap_msg.header.stamp = this->get_clock()->now();

  // RCLCPP_INFO(this->get_logger(), "Grid size: %zu", grid_data.size());
  // int sum = 0;
  // for (int8_t v : grid_data) sum += v;
  // RCLCPP_INFO(this->get_logger(), "Sum of grid values: %d", sum);

  occupancy_grid_pub_->publish(costmap_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}