#include "planner_node.hpp"
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

PlannerNode::PlannerNode()
    : Node("planner_node"), planner_(robot::PlannerCore(this->get_logger())) {
  this->declare_parameter<std::string>("world_frame", "sim_world");
  this->declare_parameter<double>("goal_tolerance", 0.5);
  this->declare_parameter<int>("replan_period_ms", 500);
  this->declare_parameter<int>("lethal_cost", 100);

  world_frame_ = this->get_parameter("world_frame").as_string();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  const int replan_period_ms = this->get_parameter("replan_period_ms").as_int();
  planner_.setLethalCost(
      static_cast<int>(this->get_parameter("lethal_cost").as_int()));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->mapCallback(msg);
      });
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        this->goalCallback(msg);
      });
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odomCallback(msg);
      });
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(replan_period_ms), [this]() {
        this->timerCallback();
      });
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  const int height = static_cast<int>(msg->info.height);
  const int width = static_cast<int>(msg->info.width);

  robot::MapGrid map;
  map.cells.assign(height, std::vector<int>(width, 0));
  for (int j{0}; j < height; ++j) {
    for (int i{0}; i < width; ++i) {
      map.cells[j][i] = msg->data[j * width + i];
    }
  }
  map.resolution = msg->info.resolution;
  map.origin_x = msg->info.origin.position.x;
  map.origin_y = msg->info.origin.position.y;
  planner_.setMap(std::move(map));

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planAndPublish();
  }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_x_ = msg->point.x;
  goal_y_ = msg->point.y;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planAndPublish();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }
  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }
  planAndPublish();
}

bool PlannerNode::goalReached() const {
  const double dx = goal_x_ - robot_x_;
  const double dy = goal_y_ - robot_y_;
  return (dx * dx + dy * dy) < (goal_tolerance_ * goal_tolerance_);
}

void PlannerNode::planAndPublish() {
  if (!goal_received_ || !odom_received_ || !planner_.hasMap()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "cannot plan: goal=%d odom=%d map=%d", goal_received_,
                         odom_received_, planner_.hasMap());
    return;
  }

  std::vector<std::pair<double, double>> waypoints;
  if (!planner_.planPath(robot_x_, robot_y_, goal_x_, goal_y_, waypoints)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "no path to goal (%.2f, %.2f)", goal_x_, goal_y_);
    return;
  }

  auto path = nav_msgs::msg::Path();
  path.header.stamp = this->now();
  path.header.frame_id = world_frame_;
  path.poses.reserve(waypoints.size());
  for (const auto &wp : waypoints) {
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header = path.header;
    pose.pose.position.x = wp.first;
    pose.pose.position.y = wp.second;
    pose.pose.orientation.w = 1.0;
    path.poses.emplace_back(pose);
  }

  path_pub_->publish(path);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
