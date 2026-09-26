#include "control_node.hpp"
#include <algorithm>
#include <chrono>
#include <utility>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"

ControlNode::ControlNode()
    : Node("control_node"), control_(robot::ControlCore(this->get_logger())) {
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("goal_tolerance", 0.1);
  this->declare_parameter<double>("linear_speed", 0.5);
  this->declare_parameter<double>("max_angular_speed", 1.5);
  this->declare_parameter<int>("control_period_ms", 100);

  control_.initialize(this->get_parameter("lookahead_distance").as_double(),
                      this->get_parameter("goal_tolerance").as_double(),
                      this->get_parameter("linear_speed").as_double());
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  const int control_period_ms =
      this->get_parameter("control_period_ms").as_int();

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        this->pathCallback(msg);
      });
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odomCallback(msg);
      });
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms), [this]() {
        this->controlLoop();
      });
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  std::vector<std::pair<double, double>> path;
  path.reserve(msg->poses.size());
  for (const auto &pose : msg->poses) {
    path.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  control_.setPath(std::move(path));
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  robot_yaw_ = tf2::getYaw(q);
  odom_received_ = true;
}

void ControlNode::publishStop() {
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void ControlNode::controlLoop() {
  if (!odom_received_ || !control_.hasPath()) {
    publishStop();
    return;
  }

  if (control_.goalReached(robot_x_, robot_y_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "goal reached, holding");
    publishStop();
    return;
  }

  double linear = 0.0;
  double angular = 0.0;
  if (!control_.computeVelocity(robot_x_, robot_y_, robot_yaw_, linear,
                               angular)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "no lookahead point, stopping");
    publishStop();
    return;
  }

  auto cmd = geometry_msgs::msg::Twist();
  cmd.linear.x = linear;
  cmd.angular.z =
      std::clamp(angular, -max_angular_speed_, max_angular_speed_);
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
