#include "map_memory_core.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger) : logger_(logger) {}

bool MapMemoryCore::costmapUpdated() const { return costmap_updated_; }
bool MapMemoryCore::shouldUpdateMap() const { return should_update_map_; }
double MapMemoryCore::getOriginX() const { return origin_x_; }
double MapMemoryCore::getOriginY() const { return origin_y_; }
void MapMemoryCore::setShouldUpdateMap(bool state) {
  should_update_map_ = state;
}
void MapMemoryCore::setCostmapUpdated(bool state) { costmap_updated_ = state; }
void MapMemoryCore::updateCostmapGrid(LocalGrid costmap) {
  latest_costmap_ = std::move(costmap);
}
void MapMemoryCore::fuse() {
  const auto &local = latest_costmap_;
  if (local.resolution > resolution_) {
    RCLCPP_WARN_ONCE(logger_,
                     "costmap resolution %.3f is coarser than map %.3f; fusion "
                     "will leave holes",
                     local.resolution, resolution_);
  }

  const double cos_yaw = std::cos(robot_yaw_);
  const double sin_yaw = std::sin(robot_yaw_);
  const int local_height = static_cast<int>(local.cells.size());
  const int local_width = static_cast<int>(local.cells[0].size());
  for (int ly{0}; ly < local_height; ++ly) {
    for (int lx{0}; lx < local_width; ++lx) {
      if (local.cells[ly][lx] > 0) {
        double px = (lx + 0.5) * local.resolution + local.origin_x;
        double py = (ly + 0.5) * local.resolution + local.origin_y;

        double wx = robot_x_ + px * cos_yaw - py * sin_yaw;
        double wy = robot_y_ + px * sin_yaw + py * cos_yaw;

        const int gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
        const int gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));

        if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
          global_grid_[gy][gx] =
              std::max(global_grid_[gy][gx], local.cells[ly][lx]);
        }
      }
    }
  }
}
void MapMemoryCore::integrateCostmap() {
  fuse();
  last_x_ = robot_x_;
  last_y_ = robot_y_;
}
const std::vector<std::vector<int>> &MapMemoryCore::getGrid() const {
  return this->global_grid_;
}
void MapMemoryCore::initialize(int width, int height, double resolution,
                               double distance_threshold) {
  this->width_ = width;
  this->height_ = height;
  this->resolution_ = resolution;
  this->distance_threshold_ = distance_threshold;

  this->origin_x_ = -(width * resolution) / 2;
  this->origin_y_ = -(height * resolution) / 2;
  this->global_grid_.assign(height_, std::vector<int>(width_, 0));
}

void MapMemoryCore::updateRobotPose(double x, double y, double yaw) {
  robot_x_ = x;
  robot_y_ = y;
  robot_yaw_ = yaw;
  if ((((x - last_x_) * (x - last_x_)) + ((y - last_y_) * (y - last_y_))) >=
      (distance_threshold_ * distance_threshold_)) {
    should_update_map_ = true;
  }
}
} // namespace robot
