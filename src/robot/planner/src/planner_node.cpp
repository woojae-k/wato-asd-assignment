#include "planner_node.hpp"

PlannerNode::PlannerNode() 
: Node("planner_node"), // Changed node name slightly to avoid conflicts
  planner_(this->get_logger()) // <-- Updated: Passing get_logger()
{
  RCLCPP_INFO(this->get_logger(), "PlannerNode starting up...");

  // QoS settings - reliable for map and goal, best-effort for odom
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", reliable_qos, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", reliable_qos, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", best_effort_qos, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", reliable_qos);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(250), std::bind(&PlannerNode::timerCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "PlannerNode initialized successfully.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  //RCLCPP_INFO(this->get_logger(), "Map received.");
  planner_.setMap(*msg);

  // If we get a new map while actively navigating, replan
  if (planner_.getState() == robot::State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    RCLCPP_INFO(this->get_logger(), "Map updated, replanning...");
    triggerPlan();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "New goal received.");
  planner_.setGoal(*msg);
  triggerPlan(); // Plan immediately upon receiving a new goal
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  planner_.setPose(*msg);
}

void PlannerNode::timerCallback() {
  // The timer's only job is to check if we've arrived
  if (planner_.getState() == robot::State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (planner_.isGoalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      planner_.resetGoal();
      
      // Publish an empty path to clear any old visualizations
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->get_clock()->now();
      empty_path.header.frame_id = "map"; // Or get from planner
      path_pub_->publish(empty_path);
    }
    // No "else" for replanning here. Replanning is triggered by
    // new goals or new maps.
  }
}

void PlannerNode::triggerPlan() {
  auto path_result = planner_.planPath();
  
  if (path_result.has_value()) {
    RCLCPP_INFO(this->get_logger(), "Publishing new path.");
    path_pub_->publish(path_result.value());
  } else {
    RCLCPP_WARN(this->get_logger(), "Planning failed, no path published.");
  }
}

// --- Main Function ---
// (This should be in its own file, e.g., planner_main.cpp,
// but I'll include it here for completeness if you're compiling this file directly)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}