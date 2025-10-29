#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"


/**
 * @class CostmapCore
 * @brief Manages the 2D occupancy grid data and logic.
 *
 * This class is independent of ROS and handles grid initialization,
 * coordinate conversions, obstacle marking, and inflation.
 */
class CostmapCore {
  public:
    /**
     * @brief Constructor
     * @param resolution Map resolution in meters/cell
     * @param width Map width in cells
     * @param height Map height in cells
     * @param origin_x Map origin x-coordinate in meters
     * @param origin_y Map origin y-coordinate in meters
     * @param inflation_radius Radius for obstacle inflation in meters
     */
    explicit CostmapCore(
      double resolution, int width, int height, double origin_x, double origin_y,
      double inflation_radius);

    /**
     * @brief Resets the costmap to its default value (free space).
     */
    void initialize();

    /**
     * @brief Converts world coordinates (meters) to grid indices (cells).
     * @param world_x X-coordinate in the world frame (meters)
     * @param world_y Y-coordinate in the world frame (meters)
     * @param grid_x Output: X-index in the grid
     * @param grid_y Output: Y-index in the grid
     * @return true if the world coordinate is within the grid bounds, false otherwise
     */
    bool worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const;

    /**
     * @brief Marks a specific cell as an obstacle.
     * @param grid_x X-index of the cell
     * @param grid_y Y-index of the cell
     */
    void markObstacle(int grid_x, int grid_y);

    /**
     * @brief Inflates all marked obstacles according to the inflation_radius.
     */
    void inflateObstacles();

    /**
     * @brief Flattens the 2D grid into a 1D vector for publishing.
     * @return A 1D vector of costmap data (row-major order).
     */
    std::vector<int8_t> getGridData() const;

    // --- Getters for map info ---
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

    // --- Map Parameters ---
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double inflation_radius_;

    // --- Constants ---
    const int8_t MAX_COST = 100;
    const int8_t FREE_SPACE = 0;

    // --- Grids ---
    // The main grid holding the final cost values (0-100)
    std::vector<std::vector<int8_t>> grid_;
    // A temporary grid to store inflation costs before merging
    std::vector<std::vector<double>> inflation_grid_;
};

#endif  // COSTMAP_CORE_HPP_