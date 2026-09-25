#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot {

// A costmap and the geometry needed to place its cells in the world
struct LocalGrid {
  std::vector<std::vector<int>> cells;
  double resolution{};
  double origin_x{};
  double origin_y{};
};

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger &logger);

  void initialize(int width, int height, double resolution,
                  double distance_threshold);

  const std::vector<std::vector<int>> &getGrid() const;
  double getOriginX() const;
  double getOriginY() const;
  void updateRobotPose(double x, double y, double yaw);
  bool shouldUpdateMap() const;
  bool costmapUpdated() const;
  void setShouldUpdateMap(bool state);
  void setCostmapUpdated(bool state);
  void integrateCostmap();
  void updateCostmapGrid(LocalGrid costmap);

private:
  void fuse();

  rclcpp::Logger logger_;

  std::vector<std::vector<int>> global_grid_;
  double resolution_{};
  int width_{};
  int height_{};
  double origin_x_{}, origin_y_{};
  double last_x_{0}, last_y_{0};
  double distance_threshold_{};
  bool costmap_updated_{false};
  double robot_x_{}, robot_y_{},
      robot_yaw_{}; // current pose, every odom message

  LocalGrid latest_costmap_;
  bool should_update_map_{true};
};

} // namespace robot

#endif
