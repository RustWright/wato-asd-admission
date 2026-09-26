#include "control_core.hpp"
#include <cmath>
#include <optional>
#include <utility>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger &logger) : logger_(logger) {}

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

std::optional<std::pair<double, double>>
ControlCore::findLookaheadPoint(double robot_x, double robot_y) const {

  for (auto point : path_) {
    double distance =
        std::sqrt((point.first - robot_x) * (point.first - robot_x) +
                  (point.second - robot_y) * (point.second - robot_y));
    if (distance >= lookahead_distance_) {
      return point;
    }
  }

  if (path_.empty()) {
    return std::nullopt;
  }

  return path_.back();
}

bool ControlCore::computeVelocity(double robot_x, double robot_y,
                                  double robot_yaw, double &linear_out,
                                  double &angular_out) {

  std::optional<std::pair<double, double>> target =
      findLookaheadPoint(robot_x, robot_y);
  if (target.has_value()) {
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    double dx = target.value().first - robot_x;
    double dy = target.value().second - robot_y;
    double cross = -dx * sin_yaw + dy * cos_yaw;
    const double distance = std::hypot(dx, dy);
    const double radius = distance * distance / (2 * cross);
    linear_out = linear_speed_;
    angular_out = linear_speed_ / radius;
    return true;
  }

  return false;
}

} // namespace robot
