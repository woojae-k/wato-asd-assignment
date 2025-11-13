#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(
  double dist_thresh, double map_w, double map_h, double map_res, double map_ox, double map_oy)
  : dist_thresh_(dist_thresh),
  map_w_(map_w),
  map_h_(map_h),
  map_res_(map_res),
  map_ox_(map_ox),
  map_oy_(map_oy)
{
  // Initialize the global map structure
  global_map_.header.frame_id = "sim_world"; // Or "odom", depending on your global frame
  global_map_.info.resolution = map_res_;
  global_map_.info.width = static_cast<uint32_t>(map_w_ / map_res_);
  global_map_.info.height = static_cast<uint32_t>(map_h_ / map_res_);
  global_map_.info.origin.position.x = map_ox_;
  global_map_.info.origin.position.y = map_oy_;
  global_map_.info.origin.orientation.w = 1.0; // No rotation

  // Initialize all cells to "unknown" (-1)
  global_map_.data.resize(global_map_.info.width * global_map_.info.height, 0);

  RCLCPP_INFO(rclcpp::get_logger("map_memory_core"), "Global map initialized: %u x %u cells @ %.2f m/res",
    global_map_.info.width, global_map_.info.height, global_map_.info.resolution);
}

// --- REFACTORED ODOM CALLBACK ---
void MapMemoryCore::processOdometry(double x, double y, double z, double w)
{
  current_x_ = x;
  current_y_ = y;
  current_theta_ = 2.0 * std::atan2(z,w);

  if (!odom_received_) {
    // This is the first odom message, set initial position
    last_update_x_ = current_x_;
    last_update_y_ = current_y_;
    odom_received_ = true;
  }

  // Compute distance traveled since last update
  double distance = std::sqrt(std::pow(current_x_ - last_update_x_, 2) + std::pow(current_y_ - last_update_y_, 2));

  if (distance >= dist_thresh_) {
    should_update_map_ = true;
    last_update_x_ = current_x_;
    last_update_y_ = current_y_;
  }
}

// --- REFACTORED COSTMAP CALLBACK ---
void MapMemoryCore::processCostmap(const nav_msgs::msg::OccupancyGrid& msg)
{
  latest_costmap_ = msg;
  costmap_received_ = true;
}

bool MapMemoryCore::shouldUpdate() const
{
  return should_update_map_ && costmap_received_;
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() const
{
  return global_map_;
}

void MapMemoryCore::resetUpdateFlag()
{
  should_update_map_ = false;
}

void MapMemoryCore::integrateCostmap()
{
  // 1. Check if costmap is available
  if (!costmap_received_) {
    return;
  }

  // 2. Retrieve map info
  double global_res = global_map_.info.resolution;
  double global_ox = global_map_.info.origin.position.x;
  double global_oy = global_map_.info.origin.position.y;
  uint32_t global_w = global_map_.info.width;
  uint32_t global_h = global_map_.info.height;

  double local_res = latest_costmap_.info.resolution;
  double local_ox = latest_costmap_.info.origin.position.x;
  double local_oy = latest_costmap_.info.origin.position.y;
  uint32_t local_w = latest_costmap_.info.width;
  uint32_t local_h = latest_costmap_.info.height;

  // 3. Precompute rotation terms for INVERSE transform
  // (cos(-theta) = cos(theta), sin(-theta) = -sin(theta))
  double cos_t = std::cos(current_theta_);
  double sin_t = std::sin(current_theta_); // sin(theta)

  // 4. Find the Bounding Box of the local map in the global frame
  // We transform the 4 corners of the local map to the global map
  
  // Local map corner coordinates (in robot frame)
  double local_x_min = local_ox;
  double local_x_max = local_ox + local_w * local_res;
  double local_y_min = local_oy;
  double local_y_max = local_oy + local_h * local_res;

  // Transform corners to global frame
  auto transform = [&](double lx, double ly) {
    return std::make_pair(
      current_x_ + lx * cos_t - ly * sin_t,
      current_y_ + lx * sin_t + ly * cos_t
    );
  };

  auto [g_x1, g_y1] = transform(local_x_min, local_y_min);
  auto [g_x2, g_y2] = transform(local_x_max, local_y_min);
  auto [g_x3, g_y3] = transform(local_x_max, local_y_max);
  auto [g_x4, g_y4] = transform(local_x_min, local_y_max);

  // Find min/max grid indices from the transformed corners
  // Note: These are world coordinates, not ints.
  double min_gx_world = std::min({g_x1, g_x2, g_x3, g_x4});
  double max_gx_world = std::max({g_x1, g_x2, g_x3, g_x4});
  double min_gy_world = std::min({g_y1, g_y2, g_y3, g_y4});
  double max_gy_world = std::max({g_y1, g_y2, g_y3, g_y4});
  
  // Convert from world coords to grid indices (and add a 1-cell buffer)
  // Clamping to global map boundaries
  int min_gx_idx = std::max(0, static_cast<int>(std::floor((min_gx_world - global_ox) / global_res)) - 1);
  int max_gx_idx = std::min(static_cast<int>(global_w - 1), static_cast<int>(std::ceil((max_gx_world - global_ox) / global_res)) + 1);
  int min_gy_idx = std::max(0, static_cast<int>(std::floor((min_gy_world - global_oy) / global_res)) - 1);
  int max_gy_idx = std::min(static_cast<int>(global_h - 1), static_cast<int>(std::ceil((max_gy_world - global_oy) / global_res)) + 1);


  // 5. Iterate through all GLOBAL map cells within the bounding box
  for (int gy = min_gy_idx; gy <= max_gy_idx; ++gy) {
    for (int gx = min_gx_idx; gx <= max_gx_idx; ++gx) {
      
      // 6. Find this global cell's center in global coordinates
      double global_x = global_ox + (gx + 0.5) * global_res;
      double global_y = global_oy + (gy + 0.5) * global_res;

      // 7. Apply INVERSE transform: Global Coords -> Local Coords
      // Step 7a: Translate relative to robot pose
      double dx = global_x - current_x_;
      double dy = global_y - current_y_;
      
      // Step 7b: Rotate back to robot's frame
      double local_x = dx * cos_t + dy * sin_t;  // cos(-theta) = cos(theta)
      double local_y = -dx * sin_t + dy * cos_t; // sin(-theta) = -sin(theta)

      // 8. Find corresponding cell index in the local map
      int cx = static_cast<int>(std::floor((local_x - local_ox) / local_res));
      int cy = static_cast<int>(std::floor((local_y - local_oy) / local_res));

      // 9. Check if this local index is within bounds of the local map
      if (cx < 0 || cx >= static_cast<int>(local_w) ||
          cy < 0 || cy >= static_cast<int>(local_h))
      {
        continue; // This global cell is outside the local map's view
      }

      // 10. Read the value from the local map
      size_t local_idx = cy * local_w + cx;
      int8_t local_val = latest_costmap_.data[local_idx];

      // 11. update the global map if the local map sees a new OBSTACLE.
      if (local_val > 0) {
        size_t global_idx = gy * global_w + gx;
        global_map_.data[global_idx] = local_val;
      }
    }
  }
}

} // namespace robot