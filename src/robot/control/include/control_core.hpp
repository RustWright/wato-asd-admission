#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <optional>
#include <utility>
#include <vector>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void initialize(double lookahead_distance, double goal_tolerance,
                    double linear_speed);

    void setPath(std::vector<std::pair<double, double>> path);
    bool hasPath() const;
    bool goalReached(double robot_x, double robot_y) const;

    // Returns false when no usable lookahead point exists. On success,
    // linear_out and angular_out are the commanded velocities.
    bool computeVelocity(double robot_x, double robot_y, double robot_yaw,
                         double &linear_out, double &angular_out);

  private:
    rclcpp::Logger logger_;

    std::vector<std::pair<double, double>> path_;
    double lookahead_distance_{1.0};
    double goal_tolerance_{0.1};
    double linear_speed_{0.5};

    std::optional<std::pair<double, double>> findLookaheadPoint(
        double robot_x, double robot_y) const;
};

}  // namespace robot

#endif
