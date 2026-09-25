#include "planner_core.hpp"
#include <cmath>
#include <utility>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {}

void PlannerCore::setMap(MapGrid map) { map_ = std::move(map); }

void PlannerCore::setLethalCost(int lethal_cost) { lethal_cost_ = lethal_cost; }

bool PlannerCore::hasMap() const { return !map_.cells.empty(); }

CellIndex PlannerCore::worldToGrid(double wx, double wy) const {
  return CellIndex(
      static_cast<int>(std::floor((wx - map_.origin_x) / map_.resolution)),
      static_cast<int>(std::floor((wy - map_.origin_y) / map_.resolution)));
}

void PlannerCore::gridToWorld(const CellIndex &c, double &wx, double &wy) const {
  wx = (c.x + 0.5) * map_.resolution + map_.origin_x;
  wy = (c.y + 0.5) * map_.resolution + map_.origin_y;
}

bool PlannerCore::inBounds(const CellIndex &c) const {
  if (map_.cells.empty()) {
    return false;
  }
  const int height = static_cast<int>(map_.cells.size());
  const int width = static_cast<int>(map_.cells[0].size());
  return c.x >= 0 && c.x < width && c.y >= 0 && c.y < height;
}

int PlannerCore::costAt(const CellIndex &c) const {
  return map_.cells[c.y][c.x];
}

bool PlannerCore::planPath(double start_x, double start_y, double goal_x,
                           double goal_y,
                           std::vector<std::pair<double, double>> &path_out) {
  path_out.clear();

  // TODO(efe): A* search goes here.
  //
  // Available to you: worldToGrid / gridToWorld / inBounds / costAt above,
  // and CellIndex, CellIndexHash, AStarNode, CompareF from the header.
  // lethal_cost_ is the cost at or above which a cell is impassable.
  (void)start_x;
  (void)start_y;
  (void)goal_x;
  (void)goal_y;
  return false;
}

}  // namespace robot
