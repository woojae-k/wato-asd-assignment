#include "planner_core.hpp"

namespace robot
{

inline std::vector<CellIndex> runAStar(
    const std::vector<std::vector<int>> &grid,
    const CellIndex &start,
    const CellIndex &goal,
    int height,
    int width)
{

  const int COST_SCALE = 0.7;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  open_set.emplace(start, heuristic(start, goal));

  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  g_score[start] = 0.0;

  while (!open_set.empty())
  {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal)
      return reconstructPath(came_from, current);

    for (auto &neighbor : getNeighbors(current))
    {
      if (!isInside(grid, neighbor, height, width) || !isWalkable(grid, neighbor))
        continue;
      
      // Cost to move from current to neighbor (1.0 for grid, 1.414 for diagonal)
      double step_cost = (std::abs(neighbor.x - current.x) + std::abs(neighbor.y - current.y) > 1) ? std::sqrt(2.0) : 1.0;
      double move_cost = step_cost + COST_SCALE * static_cast<double>(grid[neighbor.y][neighbor.x]);
      double tentative_g = g_score[current] + move_cost;

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor])
      {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.emplace(neighbor, f);
      }
    }
  }

  return {}; // no path found
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger), state_(robot::State::WAITING_FOR_GOAL) 
{
    RCLCPP_INFO(logger_, "PlannerCore initialized, waiting for goal."); // <-- Re-added
}

void PlannerCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
    global_map_ = map;
    map_received_ = true;
    RCLCPP_INFO(logger_, "Map received (%d x %d)", global_map_.info.width, global_map_.info.height); // <-- Re-added
}

void PlannerCore::setGoal(const geometry_msgs::msg::PointStamped& goal) {
    destination_ = goal;
    goal_received_ = true;
    state_ = robot::State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    RCLCPP_INFO(logger_, "New goal set: (%.2f, %.2f). State changed to WAITING_FOR_ROBOT_TO_REACH_GOAL.", // <-- Re-added
        destination_.point.x, destination_.point.y); // <-- Re-added
}

void PlannerCore::setPose(const nav_msgs::msg::Odometry& odom) {
    current_pose_ = odom.pose.pose;
    pose_received_ = true;
}

robot::State PlannerCore::getState() const {
    return state_;
}

void PlannerCore::resetGoal() {
    goal_received_ = false;
    state_ = robot::State::WAITING_FOR_GOAL;
    RCLCPP_INFO(logger_, "Goal cleared. State changed to WAITING_FOR_GOAL."); // <-- Re-added
}

bool PlannerCore::isGoalReached() const {
    if (!pose_received_ || !goal_received_) {
        return false;
    }

    double dx = current_pose_.position.x - destination_.point.x;
    double dy = current_pose_.position.y - destination_.point.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    return dist < goal_tolerance_;
}

bool PlannerCore::isStateValid() const {
    if (!map_received_) {
        RCLCPP_WARN(logger_, "Cannot plan: Map not yet received."); // <-- Re-added
        return false;
    }
    if (!pose_received_) {
        RCLCPP_WARN(logger_, "Cannot plan: Pose not yet received."); // <-- Re-added
        return false;
    }
    if (!goal_received_) {
        RCLCPP_WARN(logger_, "Cannot plan: Goal not yet received."); // <-- Re-added
        return false;
    }
    return true;
}

/**
 * @brief Converts world coordinates (meters) to grid cell indices.
 */
CellIndex PlannerCore::worldToGrid(double x, double y) const {
    CellIndex idx;
    idx.x = static_cast<int>((x - global_map_.info.origin.position.x) / global_map_.info.resolution);
    idx.y = static_cast<int>((y - global_map_.info.origin.position.y) / global_map_.info.resolution);
    return idx;
}

/**
 * @brief Converts grid cell indices to world coordinates (meters).
 * Places the point in the center of the cell.
 */
geometry_msgs::msg::Point PlannerCore::gridToWorld(int x, int y) const {
    geometry_msgs::msg::Point p;
    p.x = (x + 0.5) * global_map_.info.resolution + global_map_.info.origin.position.x;
    p.y = (y + 0.5) * global_map_.info.resolution + global_map_.info.origin.position.y;
    p.z = 0.0; // Assuming 2D planning
    return p;
}

/**
 * @brief Converts the 1D occupancy grid data into a 2D vector for A*.
 */
std::vector<std::vector<int>> PlannerCore::convertMapToGrid() const {
    int height = global_map_.info.height;
    int width = global_map_.info.width;
    
    std::vector<std::vector<int>> grid(height, std::vector<int>(width));
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // OccupancyGrid data is row-major
            grid[y][x] = global_map_.data[y * width + x];
        }
    }
    return grid;
}


/**
 * @brief Main A* planning function.
 */
std::optional<nav_msgs::msg::Path> PlannerCore::planPath() {
    RCLCPP_INFO(logger_, "Attempting to plan path..."); // <-- Re-added

    if (!isStateValid()) {
        return std::nullopt; // Return empty optional
    }

    // 1. Convert map
    int height = global_map_.info.height;
    int width = global_map_.info.width;
    std::vector<std::vector<int>> grid = convertMapToGrid();

    // 2. Get start and goal in grid coordinates
    CellIndex start_cell = worldToGrid(current_pose_.position.x, current_pose_.position.y);
    CellIndex goal_cell = worldToGrid(destination_.point.x, destination_.point.y);

    RCLCPP_INFO(logger_, "Planning from grid cell (%d, %d) to (%d, %d)", // <-- Re-added
        start_cell.x, start_cell.y, goal_cell.x, goal_cell.y); // <-- Re-added

    // 3. Check if start/goal are valid
    if (!isInside(grid, start_cell, height, width)) {
        RCLCPP_ERROR(logger_, "Start position (%.2f, %.2f) is outside map bounds!", // <-- Re-added
            current_pose_.position.x, current_pose_.position.y); // <-- Re-added
        return std::nullopt;
    }
     if (!isInside(grid, goal_cell, height, width)) {
        RCLCPP_ERROR(logger_, "Goal position (%.2f, %.2f) is outside map bounds!", // <-- Re-added
            destination_.point.x, destination_.point.y); // <-- Re-added
        return std::nullopt;
    }
    if (!isWalkable(grid, start_cell)) {
        RCLCPP_ERROR(logger_, "Start position (%d, %d) is on an obstacle!", start_cell.x, start_cell.y); // <-- Re-added
        return std::nullopt;
    }
     if (!isWalkable(grid, goal_cell)) {
        RCLCPP_ERROR(logger_, "Goal position (%d, %d) is on an obstacle!", goal_cell.x, goal_cell.y); // <-- Re-added
        return std::nullopt;
    }

    // 4. Run A*
    std::vector<CellIndex> cell_path = runAStar(grid, start_cell, goal_cell, height, width);

    if (cell_path.empty()) {
        RCLCPP_WARN(logger_, "No path found to the goal!"); // <-- Re-added
        return std::nullopt;
    }

    RCLCPP_INFO(logger_, "Path found with %zu points.", cell_path.size()); // <-- Re-added

    // 5. Convert cell path back to world path
    nav_msgs::msg::Path world_path;
    world_path.header.stamp = rclcpp::Clock().now();
    world_path.header.frame_id = global_map_.header.frame_id; // Path is in map frame

    for (const auto& cell : cell_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = world_path.header;
        pose.pose.position = gridToWorld(cell.x, cell.y);
        // We don't calculate orientation here, just set to neutral
        pose.pose.orientation.w = 1.0; 
        world_path.poses.push_back(pose);
    }

    return world_path; // Return the completed path
}

} // namespace robot