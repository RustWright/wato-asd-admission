#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <utility>
#include <vector>

namespace robot {

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score; // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

// ------------------------------------------------------------

// A grid and the geometry needed to place its cells in the world
struct MapGrid {
  std::vector<std::vector<int>> cells;
  double resolution{};
  double origin_x{};
  double origin_y{};
};

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger &logger);

  void setMap(MapGrid map);
  void setLethalCost(int lethal_cost);
  void setCostWeight(double cost_weight);
  bool hasMap() const;

  // Returns false when no path exists. On success, path_out holds world-frame
  // waypoints from start to goal inclusive.
  bool planPath(double start_x, double start_y, double goal_x, double goal_y,
                std::vector<std::pair<double, double>> &path_out);

  CellIndex worldToGrid(double wx, double wy) const;
  void gridToWorld(const CellIndex &c, double &wx, double &wy) const;
  bool inBounds(const CellIndex &c) const;
  int costAt(const CellIndex &c) const;

private:
  rclcpp::Logger logger_;

  MapGrid map_;
  int lethal_cost_{100};
  double cost_weight_{0.05};

  double stepCost(CellIndex start_cell, CellIndex end_cell);
};

} // namespace robot

#endif
