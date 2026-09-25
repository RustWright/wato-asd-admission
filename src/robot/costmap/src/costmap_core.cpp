#include "costmap_core.hpp"
#include <algorithm>
#include <cmath>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger &logger) : logger_(logger) {}

void CostmapCore::initialize(int width, int height, double resolution,
                             double inflation_radius, int max_cost) {
  this->width_ = width;
  this->height_ = height;
  this->resolution_ = resolution;
  this->inflation_radius_ = inflation_radius;
  this->max_cost_ = max_cost;
  this->origin_x_ = -(width * resolution) / 2;
  this->origin_y_ = -(height * resolution) / 2;
  resetGrid();
}

void CostmapCore::resetGrid() {
  this->grid_.assign(height_, std::vector<int>(width_, 0));
}

void CostmapCore::processScan(double angle_min, double angle_increment,
                              const std::vector<float> &ranges,
                              double range_min, double range_max) {
  resetGrid();

  for (size_t i{0}; i < ranges.size(); ++i) {
    double angle = angle_min + (i * angle_increment);
    double range = ranges[i];
    if (range < range_max && range > range_min) {
      int x_grid, y_grid;
      // Calculate grid coordinates
      if (convertToGrid(range, angle, x_grid, y_grid)) {
        markObstacle(x_grid, y_grid);
        inflateAroundCell(x_grid, y_grid);
      }
    }
  }
}

const std::vector<std::vector<int>> &CostmapCore::getGrid() const {
  return this->grid_;
}
double CostmapCore::getOriginX() const { return this->origin_x_; }
double CostmapCore::getOriginY() const { return this->origin_y_; }

bool CostmapCore::convertToGrid(double range, double angle, int &x_grid,
                                int &y_grid) const {
  double x_rel_robot = range * cos(angle);
  double y_rel_robot = range * sin(angle);

  double x_rel_origin = x_rel_robot - origin_x_;
  double y_rel_origin = y_rel_robot - origin_y_;

  x_grid = std::floor(x_rel_origin / resolution_);
  y_grid = std::floor(y_rel_origin / resolution_);

  return (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_)
             ? true
             : false;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  this->grid_[y_grid][x_grid] = 100;
}

void CostmapCore::inflateAroundCell(int x_grid, int y_grid) {
  int cell_radius = std::floor(inflation_radius_ / resolution_);
  for (int j{y_grid - cell_radius}; j <= y_grid + cell_radius; ++j) {
    for (int i{x_grid - cell_radius}; i <= x_grid + cell_radius; ++i) {
      double sqr_norm =
          (((i - x_grid) * (i - x_grid)) + ((j - y_grid) * (j - y_grid))) *
          (resolution_ * resolution_);
      if (i >= 0 && i < width_ && j >= 0 && j < height_ &&
          ((sqr_norm) < inflation_radius_ * inflation_radius_)) {
        int new_cost = std::lround(
            max_cost_ * (1 - (std::sqrt(sqr_norm) / inflation_radius_)));
        grid_[j][i] = std::max(grid_[j][i], new_cost);
      }
    }
  }
}

} // namespace robot
