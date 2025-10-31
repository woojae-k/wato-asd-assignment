#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(resolution_, map_width_, map_height_, origin_x_, origin_y_, inflation_radius_)) {
  // Initialize the constructs and their parameters
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "CostmapNode initialized.");
}
 
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", scan->ranges.size());

  costmap_.initialize();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    double angle = scan->angle_min + i * scan->angle_increment;

    if (range >= scan->range_min && range <= scan->range_max && std::isfinite(range)) {
      double world_x = range * std::cos(angle);
      double world_y = range * std::sin(angle);

      int grid_x, grid_y;
      if (costmap_.worldToGrid(world_x, world_y, grid_x, grid_y)) {
        costmap_.markObstacle(grid_x, grid_y);
      }
    }
  }

  costmap_.inflateObstacles();
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  costmap_msg_.header.stamp = this->get_clock()->now();
  costmap_msg_.header.frame_id = "map";

  costmap_msg_.info.resolution = resolution_;
  costmap_msg_.info.width = map_width_;
  costmap_msg_.info.height = map_height_;
  costmap_msg_.info.origin.position.x = origin_x_;
  costmap_msg_.info.origin.position.y = origin_y_;
  costmap_msg_.info.origin.orientation.w = 1.0;

  auto grid_data = costmap_.getGridData();

  // RCLCPP_INFO(this->get_logger(), "Grid size: %zu", grid_data.size());
  // int sum = 0;
  // for (int8_t v : grid_data) sum += v;
  // RCLCPP_INFO(this->get_logger(), "Sum of grid values: %d", sum);

  costmap_msg_.data = grid_data;
  occupancy_grid_pub_->publish(costmap_msg_);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}