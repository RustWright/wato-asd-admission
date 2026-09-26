#include "control_core.hpp"
#include <cmath>
#include <utility>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::initialize(double lookahead_distance, double goal_tolerance,
                             double linear_speed) {
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
}

void ControlCore::setPath(std::vector<std::pair<double, double>> path) {
  path_ = std::move(path);
}

bool ControlCore::hasPath() const { return !path_.empty(); }

bool ControlCore::goalReached(double robot_x, double robot_y) const {
  if (path_.empty()) {
    return false;
  }
  const double dx = path_.back().first - robot_x;
  const double dy = path_.back().second - robot_y;
  return (dx * dx + dy * dy) < (goal_tolerance_ * goal_tolerance_);
}

std::optional<std::pair<double, double>> ControlCore::findLookaheadPoint(
    double robot_x, double robot_y) const {
  // TODO(efe): walk path_ and return the first point at least
  // lookahead_distance_ away from the robot, or the last point when none is.
  (void)robot_x;
  (void)robot_y;
  return std::nullopt;
}

bool ControlCore::computeVelocity(double robot_x, double robot_y,
                                  double robot_yaw, double &linear_out,
                                  double &angular_out) {
  linear_out = 0.0;
  angular_out = 0.0;

  // TODO(efe): Pure Pursuit.
  //
  // 1. target = findLookaheadPoint(robot_x, robot_y); bail if nullopt.
  // 2. Express the target in the robot frame (inverse of the map_memory
  //    rotation, since that went robot frame -> world).
  // 3. Turn its lateral offset into the curvature of the arc from the robot
  //    through the target.
  // 4. linear_out from linear_speed_; angular_out from speed and curvature.
  (void)robot_yaw;
  return false;
}

}  // namespace robot
