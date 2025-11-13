#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <optional> // Used for returning the path

namespace robot
{

// ------------------- Supporting Structures -------------------
// (These are unchanged from your file)

// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// A* node
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;
  }
};

// A* Helper Functions
inline bool isInside(const std::vector<std::vector<int>> &grid, const CellIndex &idx, int height, int width)
{
  return idx.y >= 0 && idx.y < height &&
         idx.x >= 0 && idx.x < width;
}

// We consider a cell walkable if its cost is not 100(obstacle)
inline bool isWalkable(const std::vector<std::vector<int>> &grid, const CellIndex &idx)
{
  return grid[idx.y][idx.x] < 75; 
}

inline std::vector<CellIndex> getNeighbors(const CellIndex &idx)
{
  return {
      {idx.x + 1, idx.y},
      {idx.x - 1, idx.y},
      {idx.x, idx.y + 1},
      {idx.x, idx.y - 1},
      {idx.x + 1, idx.y + 1},
      {idx.x - 1, idx.y - 1},
      {idx.x + 1, idx.y - 1},
      {idx.x - 1, idx.y + 1}
  };
}

inline double heuristic(const CellIndex &a, const CellIndex &b)
{
  // Manhattan distance for 4-way movement
  // return std::abs(a.x - b.x) + std::abs(a.y - b.y);
  
  // Euclidean distance (sqrt) or Octile distance for 8-way movement
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// Reconstruct path
inline std::vector<CellIndex> reconstructPath(
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
    CellIndex current)
{
  std::vector<CellIndex> total_path = {current};
  while (came_from.find(current) != came_from.end())
  {
    current = came_from[current];
    total_path.push_back(current);
  }
  std::reverse(total_path.begin(), total_path.end());
  return total_path;
}

// ---------------------------------------------------------------------------
// A* Algorithm (core function)
// ---------------------------------------------------------------------------
inline std::vector<CellIndex> runAStar(
    const std::vector<std::vector<int>> &grid,
    const CellIndex &start,
    const CellIndex &goal,
    int height,
    int width);

// ------------------- Planner State Machine -------------------

enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };


class PlannerCore {
  public:
    // Constructor now matches your .cpp file
    explicit PlannerCore(const rclcpp::Logger& logger);

    // --- Public API for the Node ---
    void setMap(const nav_msgs::msg::OccupancyGrid& map);
    void setGoal(const geometry_msgs::msg::PointStamped& goal);
    void setPose(const nav_msgs::msg::Odometry& odom);

    robot::State getState() const;
    bool isGoalReached() const;
    void resetGoal();

    // Main planning function. Returns a path if successful.
    std::optional<nav_msgs::msg::Path> planPath();

  private:
    // --- Helper Functions ---
    CellIndex worldToGrid(double x, double y) const;
    geometry_msgs::msg::Point gridToWorld(int x, int y) const;
    std::vector<std::vector<int>> convertMapToGrid() const;
    bool isStateValid() const; // Checks if we have map, pose, and goal

    // --- Member Variables ---
    rclcpp::Logger logger_; // <-- Re-added
    robot::State state_;
    
    // Store received data
    nav_msgs::msg::OccupancyGrid global_map_;
    geometry_msgs::msg::PointStamped destination_;
    geometry_msgs::msg::Pose current_pose_; // Just the pose, not full odom

    // State flags
    bool map_received_ = false;
    bool pose_received_ = false;
    bool goal_received_ = false;

    // Goal tolerance
    double goal_tolerance_ = 0.25; // meters
};

}  // namespace robot

#endif