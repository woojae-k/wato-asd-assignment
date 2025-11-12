#include "map_memory_node.hpp"

// Constructor: Initializes parameters, then the mapmemory_ object
MapMemoryNode::MapMemoryNode()
  : Node("map_memory_node"), 
  mapmemory_(robot::MapMemoryCore(
    distance_threshold_,
    map_width_meters_,
    map_height_meters_,
    map_resolution_,
    map_origin_x_,
    map_origin_y_
    ))
{  
  // Initialize ROS publisher
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

  // Initialize ROS subscribers
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", sensor_qos,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, // Default QoS
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  // Initialize timer
  auto timer_period = std::chrono::duration<double>(1.0 / update_hz);
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&MapMemoryNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Map Memory Node has started.");
  RCLCPP_INFO(this->get_logger(), " - Distance Threshold: %.2f m", dist_thresh);
  RCLCPP_INFO(this->get_logger(), " - Update Frequency: %.2f Hz", update_hz);

  // initializeGlobalmap();
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract data and pass it to the core
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  mapmemory_.processOdometry(x, y, z, w);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Pass the dereferenced message (the grid object) to the core
  mapmemory_.processCostmap(*msg);
}

void MapMemoryNode::timerCallback()
{
  if (mapmemory_.shouldUpdate()) {
    RCLCPP_DEBUG(this->get_logger(), "Update conditions met. Integrating costmap.");
    
    // Perform the map fusion
    mapmemory_.integrateCostmap();

    // Get the updated map and set its timestamp
    nav_msgs::msg::OccupancyGrid global_map = mapmemory_.getGlobalMap();
    global_map.header.stamp = this->get_clock()->now();
    
    // Publish the map
    map_pub_->publish(std::move(global_map));

    // Reset the flag
    mapmemory_.resetUpdateFlag();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
