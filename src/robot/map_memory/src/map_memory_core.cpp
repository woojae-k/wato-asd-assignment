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
    global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);

    RCLCPP_INFO(rclcpp::get_logger("map_memory_core"), "Global map initialized: %u x %u cells @ %.2f m/res",
        global_map_.info.width, global_map_.info.height, global_map_.info.resolution);
}

// --- REFACTORED ODOM CALLBACK ---
void MapMemoryCore::processOdometry(double x, double y)
{
    current_x_ = x;
    current_y_ = y;

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
    if (!costmap_received_) {
        return; // No costmap to integrate
    }

    // Get origins and resolution
    double global_res = global_map_.info.resolution;
    double global_ox = global_map_.info.origin.position.x;
    double global_oy = global_map_.info.origin.position.y;

    double local_res = latest_costmap_.info.resolution;
    double local_ox = latest_costmap_.info.origin.position.x;
    double local_oy = latest_costmap_.info.origin.position.y;

    if (std::abs(global_res - local_res) > 1e-5) {
        RCLCPP_WARN(rclcpp::get_logger("map_memory_core"), "Resolution mismatch! Global: %.3f, Local: %.3f. Skipping update.", global_res, local_res);
        return;
    }

    // Iterate over every cell in the *local* costmap
    for (uint32_t cy = 0; cy < latest_costmap_.info.height; ++cy) {
        for (uint32_t cx = 0; cx < latest_costmap_.info.width; ++cx) {
            
            // 1. Get value from local costmap
            size_t local_idx = cy * latest_costmap_.info.width + cx;
            int8_t local_val = latest_costmap_.data[local_idx];

            if (local_val == -1) {
                continue;
            }

            // 2. Find world coordinates of the local cell's center
            double world_x = local_ox + (cx + 0.5) * local_res;
            double world_y = local_oy + (cy + 0.5) * local_res;

            // 3. Find the corresponding cell index in the *global* map
            int gx = static_cast<int>(std::floor((world_x - global_ox) / global_res));
            int gy = static_cast<int>(std::floor((world_y - global_oy) / global_res));

            // 4. Check if the cell is within the bounds of the global map
            if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) &&
                gy >= 0 && gy < static_cast<int>(global_map_.info.height))
            {
                // 5. Update the global map
                size_t global_idx = gy * global_map_.info.width + gx;
                global_map_.data[global_idx] = local_val;
            }
        }
    }
}


} // namespace robot