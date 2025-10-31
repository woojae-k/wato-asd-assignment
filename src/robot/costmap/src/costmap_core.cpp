#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(
  double resolution, int width, int height, double origin_x, double origin_y,
  double inflation_radius)
: resolution_(resolution),
  width_(width),
  height_(height),
  origin_x_(origin_x),
  origin_y_(origin_y),
  inflation_radius_(inflation_radius)
{
  // Resize the grids to the specified dimensions
  grid_.resize(height_, std::vector<int8_t>(width_));
  inflation_grid_.resize(height_, std::vector<double>(width_));

  // Initialize the costmap
  initialize();
}

void CostmapCore::initialize()
{
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      grid_[y][x] = FREE_SPACE;
      inflation_grid_[y][x] = 0.0;
    }
  }
}

bool CostmapCore::worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const
{
  // Apply origin offset and divide by resolution
  grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
  grid_y = static_cast<int>((world_y - origin_y_) / resolution_);

  // Check if the resulting indices are within the grid bounds
  return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

void CostmapCore::markObstacle(int grid_x, int grid_y)
{
  // Check bounds before marking
  if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
    grid_[grid_y][grid_x] = MAX_COST;
  }
}

double CostmapCore::computeInflationCost(double distance) const
{
  if (distance >= inflation_radius_) {
    return 0.0;
  }
  // Formula: cost = max_cost * (1 - distance / inflation_radius)
  return static_cast<double>(MAX_COST) * (1.0 - distance / inflation_radius_);
}

void CostmapCore::inflateObstacles()
{
  // Calculate the inflation window size in cells
  const int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

  // Pass 1: Calculate inflation costs in the temporary grid
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      // If this cell is an obstacle, inflate around it
      if (grid_[y][x] == MAX_COST) {
        // Iterate in a square window around the obstacle
        for (int iy = -inflation_cells; iy <= inflation_cells; ++iy) {
          for (int ix = -inflation_cells; ix <= inflation_cells; ++ix) {
            int ny = y + iy;
            int nx = x + ix;

            // Check bounds of the neighbor cell
            if (ny >= 0 && ny < height_ && nx >= 0 && nx < width_) {
              // Calculate distance in meters
              double distance = std::hypot(ix * resolution_, iy * resolution_);

              // If within radius, calculate cost
              if (distance <= inflation_radius_) {
                double cost = computeInflationCost(distance);
                // Update the temp grid only if the new cost is higher
                if (cost > inflation_grid_[ny][nx]) {
                  inflation_grid_[ny][nx] = cost;
                }
              }
            }
          }
        }
      }
    }
  }

  // Pass 2: Merge the inflation grid into the main grid
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      // Only update if not already a full obstacle
      if (grid_[y][x] != MAX_COST) {
        grid_[y][x] = static_cast<int8_t>(inflation_grid_[y][x]);
      }
    }
  }
}

std::vector<int8_t> CostmapCore::getGridData() const
{
  std::vector<int8_t> data(width_ * height_);
  // Flatten the 2D grid into a 1D vector (row-major order)
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      data[y * width_ + x] = grid_[y][x];
    }
  }
  return data;
}

}