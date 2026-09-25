#include "planner_core.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {}

void PlannerCore::setMap(MapGrid map) { map_ = std::move(map); }

void PlannerCore::setLethalCost(int lethal_cost) { lethal_cost_ = lethal_cost; }

void PlannerCore::setCostWeight(double cost_weight) {
  cost_weight_ = cost_weight;
}

bool PlannerCore::hasMap() const { return !map_.cells.empty(); }

CellIndex PlannerCore::worldToGrid(double wx, double wy) const {
  return CellIndex(
      static_cast<int>(std::floor((wx - map_.origin_x) / map_.resolution)),
      static_cast<int>(std::floor((wy - map_.origin_y) / map_.resolution)));
}

void PlannerCore::gridToWorld(const CellIndex &c, double &wx,
                              double &wy) const {
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

double PlannerCore::stepCost(CellIndex start_cell, CellIndex end_cell) {
  return std::sqrt(std::pow(start_cell.x - end_cell.x, 2.0) +
                   std::pow(start_cell.y - end_cell.y, 2.0));
}

bool PlannerCore::planPath(double start_x, double start_y, double goal_x,
                           double goal_y,
                           std::vector<std::pair<double, double>> &path_out) {
  path_out.clear();

  // create start node and goal node
  CellIndex start_node_idx = worldToGrid(start_x, start_y);
  AStarNode start_node = AStarNode(start_node_idx, 0.0);

  CellIndex goal_idx = worldToGrid(goal_x, goal_y);

  // create open set of nodes
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_set<CellIndex, CellIndexHash> closed;
  std::unordered_map<CellIndex, double, CellIndexHash>
      best_g; // for choosing the shortest path and updating f
  std::unordered_map<CellIndex, CellIndex, CellIndexHash>
      parent_map; // for recreating the path at completion

  open.push(start_node);
  best_g[start_node_idx] = 0.0;

  while (!open.empty()) {
    const CellIndex current = open.top().index;
    open.pop();
    if (closed.count(current)) {
      continue;
    }
    closed.insert(current);

    if (current == goal_idx) {
      CellIndex parent = current;
      double wx, wy;
      while (parent != start_node_idx) {
        gridToWorld(parent, wx, wy);
        path_out.push_back({wx, wy});
        parent = parent_map.at(parent);
      }

      gridToWorld(start_node_idx, wx, wy);
      path_out.push_back({wx, wy});

      std::reverse(path_out.begin(), path_out.end());
      return true;
    }

    for (int j{-1}; j <= 1; ++j) {
      for (int i{-1}; i <= 1; ++i) {
        if (i == 0 && j == 0) {
          continue;
        }
        CellIndex neighbour = CellIndex(current.x + i, current.y + j);
        if ((closed.count(neighbour) > 0) || !(inBounds(neighbour)) ||
            (costAt(neighbour) >= lethal_cost_)) {
          continue;
        }
        double g_score = best_g.at(current) + stepCost(current, neighbour) +
                         cost_weight_ * costAt(neighbour);

        if (!best_g.count(neighbour) || g_score < best_g.at(neighbour)) {
          best_g[neighbour] = g_score;
          double h_score = stepCost(goal_idx, neighbour);
          double f_score = g_score + h_score;
          parent_map[neighbour] = current;
          open.push(AStarNode(neighbour, f_score));
        }
      }
    }
  }

  return false;
}

} // namespace robot
