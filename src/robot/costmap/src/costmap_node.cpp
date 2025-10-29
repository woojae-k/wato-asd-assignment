#include "costmap_node.hpp"

CostmapNode::CostmapNode(const rclcpp::NodeOptions & options)
: Node("costmap_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing CostmapNode...");

  core_ = std::make_unique<CostmapCore>(
    resolution_, map_width_, map_height_, origin_x_, origin_y_, inflation_radius_);

  // costmap message
  costmap_msg_.header.frame_id = "map";
  costmap_msg_.info.resolution = core_->getResolution();
  costmap_msg_.info.width = core_->getWidth();
  costmap_msg_.info.height = core_->getHeight();
  costmap_msg_.info.origin.position.x = core_->getOriginX();
  costmap_msg_.info.origin.position.y = core_->getOriginY();
  costmap_msg_.info.origin.orientation.w = 1.0;
  costmap_msg_.data.resize(core_->getWidth() * core_->getHeight());

  // Publisher
  pub_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // Subscriber
  auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", sub_qos, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "CostmapNode initialized. Waiting for /lidar scans...");
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  core_->initialize();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    double angle = scan->angle_min + i * scan->angle_increment;

    if (range >= scan->range_min && range <= scan->range_max && std::isfinite(range)) {
      double world_x = range * std::cos(angle);
      double world_y = range * std::sin(angle);

      int grid_x, grid_y;
      if (core_->worldToGrid(world_x, world_y, grid_x, grid_y)) {
        core_->markObstacle(grid_x, grid_y);
      }
    }
  }

  core_->inflateObstacles();
  publishCostmap();
}

void CostmapNode::publishCostmap()
{
  costmap_msg_.header.stamp = this->get_clock()->now();
  costmap_msg_.data = core_->getGridData();
  pub_costmap_->publish(costmap_msg_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
