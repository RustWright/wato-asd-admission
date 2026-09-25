#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot {

class CostmapCore {
public:
  // Constructor, we pass in the node's RCLCPP logger to enable logging to
  // terminal
  explicit CostmapCore(const rclcpp::Logger &logger);

  void initialize(int width, int height, double resolution,
                  double inflation_radius, int max_cost);

  void processScan(double angle_min, double angle_increment,
                   const std::vector<float> &ranges, double range_min,
                   double range_max);

  const std::vector<std::vector<int>> &getGrid() const;
  double getOriginX() const;
  double getOriginY() const;

private:
  rclcpp::Logger logger_;

  std::vector<std::vector<int>> grid_{};

  double resolution_{};
  int width_{};
  int height_{};
  double origin_x_{}; // origin x is calculable from width and resolution
  double origin_y_{}; // origin y is calculable from height and resolution
  double inflation_radius_{};
  int max_cost_{};

  void resetGrid();
  bool convertToGrid(double range, double angle, int &x_grid,
                     int &y_grid) const;
  void markObstacle(int x_grid, int y_grid);
  void inflateAroundCell(int x_grid, int y_grid);
};

} // namespace robot

#endif
