#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(
      double resolution, int width, int height, double origin_x, double origin_y,
      double inflation_radius);

    void initialize();

    void processLidar(const sensor_msgs::msg::LaserScan::SharedPtr &msg);

    bool worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const;

    void markObstacle(int grid_x, int grid_y);

    void inflateObstacles();

    std::vector<int8_t> processGridData() const;
  
    // nav_msgs::msg::OccupancyGrid getCostmap() const {return costmap_msg_;}
    nav_msgs::msg::OccupancyGrid getCostmap();

    int getWidth() const {return width_;}
    int getHeight() const {return height_;}
    double getResolution() const {return resolution_;}
    double getOriginX() const {return origin_x_;}
    double getOriginY() const {return origin_y_;}

  private:

    /**
     * @brief Computes the inflation cost based on distance.
     * @param distance Distance from the nearest obstacle
     * @return The cost value (0-100)
     */
    double computeInflationCost(double distance) const;

    // Map Parameters 
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double inflation_radius_;

    // Constants 
    const int8_t MAX_COST = 100;
    const int8_t FREE_SPACE = 0;
    
    // Message
    nav_msgs::msg::OccupancyGrid costmap_msg_;

    // Grids
    // The main grid holding the final cost values (0-100)
    std::vector<std::vector<int8_t>> grid_;
    // A temporary grid to store inflation costs before merging
    std::vector<std::vector<double>> inflation_grid_;
};

}  

#endif  