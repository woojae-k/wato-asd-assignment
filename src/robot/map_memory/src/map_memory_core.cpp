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

    // 3. Retrieve map info
    double global_res = global_map_.info.resolution;
    double global_ox = global_map_.info.origin.position.x;
    double global_oy = global_map_.info.origin.position.y;

    double local_res = latest_costmap_.info.resolution;
    double local_ox = latest_costmap_.info.origin.position.x;
    double local_oy = latest_costmap_.info.origin.position.y;

    uint32_t local_w = latest_costmap_.info.width;
    uint32_t local_h = latest_costmap_.info.height;
    uint32_t global_w = global_map_.info.width;
    uint32_t global_h = global_map_.info.height;

    // 4. Precompute rotation terms
    double cos_t = std::cos(current_theta_);
    double sin_t = std::sin(current_theta_);

    // 5. Iterate through all cells in the local costmap
    for (uint32_t cy = 0; cy < local_h; ++cy) {
        for (uint32_t cx = 0; cx < local_w; ++cx) {
            size_t local_idx = cy * local_w + cx;
            int8_t local_val = latest_costmap_.data[local_idx];

            // Skip unknown cells
            if (local_val == -1) {
                continue;
            }

            // 6. Compute local cell center
            double local_x = local_ox + (cx + 0.5) * local_res;
            double local_y = local_oy + (cy + 0.5) * local_res;

            // 7. Transform local → global using robot pose
            double global_x = current_x_ + local_x * cos_t - local_y * sin_t;
            double global_y = current_y_ + local_x * sin_t + local_y * cos_t;

            // 8. Find the corresponding cell index in the global map
            int gx = static_cast<int>(std::floor((global_x - global_ox) / global_res));
            int gy = static_cast<int>(std::floor((global_y - global_oy) / global_res));

            // 9. Check bounds
            if (gx < 0 || gx >= static_cast<int>(global_w) ||
                gy < 0 || gy >= static_cast<int>(global_h))
            {
                continue;
            }

            // 10. Update global map cell
            size_t global_idx = gy * global_w + gx;
            global_map_.data[global_idx] = local_val;
        }
    }
}


} // namespace robot