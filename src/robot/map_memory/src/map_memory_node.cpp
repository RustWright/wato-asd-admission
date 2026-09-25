#include "map_memory_node.hpp"
#include <algorithm>
#include <chrono>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"),
      map_memory_(robot::MapMemoryCore(this->get_logger())) {

  // Initialize the constructs and their parameters
  this->declare_parameter<double>("resolution", 0.2);
  this->declare_parameter<int>("width", 200);
  this->declare_parameter<int>("height", 200);
  this->declare_parameter<double>("distance_threshold", 1.5);

  double resolution = this->get_parameter("resolution").as_double();
  int width = this->get_parameter("width").as_int();
  int height = this->get_parameter("height").as_int();
  double distance_threshold =
      this->get_parameter("distance_threshold").as_double();

  map_memory_.initialize(width, height, resolution, distance_threshold);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        MapMemoryNode::costmapCallback(msg);
      });
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        MapMemoryNode::odomCallback(msg);
      });
  world_map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this, width, height, resolution]() {
        MapMemoryNode::updateMap(width, height, resolution);
      });
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  const int costmap_height = static_cast<int>(msg->info.height);
  const int costmap_width = static_cast<int>(msg->info.width);
  std::vector<std::vector<int>> grid{};
  grid.assign(costmap_height, std::vector<int>(costmap_width, 0));
  for (int j{0}; j < costmap_height; ++j) {
    for (int i{0}; i < costmap_width; ++i) {
      grid[j][i] = msg->data[j * costmap_width + i];
    }
  }

  robot::LocalGrid local;
  local.cells = std::move(grid);
  local.resolution = msg->info.resolution;
  local.origin_x = msg->info.origin.position.x;
  local.origin_y = msg->info.origin.position.y;
  map_memory_.updateCostmapGrid(std::move(local));

  map_memory_.setCostmapUpdated(true);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  map_memory_.updateRobotPose(msg->pose.pose.position.x,
                              msg->pose.pose.position.y, tf2::getYaw(q));
}

void MapMemoryNode::updateMap(int width, int height, double resolution) {
  if (map_memory_.shouldUpdateMap() && map_memory_.costmapUpdated()) {
    map_memory_.integrateCostmap();
    const auto &grid = this->map_memory_.getGrid();
    auto occupancy_grid = nav_msgs::msg::OccupancyGrid();
    occupancy_grid.header.frame_id = "sim_world";
    occupancy_grid.header.stamp = this->now();
    occupancy_grid.info.height = height;
    occupancy_grid.info.width = width;
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.origin.position.x = this->map_memory_.getOriginX();
    occupancy_grid.info.origin.position.y = this->map_memory_.getOriginY();
    occupancy_grid.data.reserve(width * height);

    for (const auto &row : grid) {
      for (const auto &val : row) {
        occupancy_grid.data.emplace_back(std::clamp(val, -1, 100));
      }
    }

    world_map_pub_->publish(occupancy_grid);
    map_memory_.setShouldUpdateMap(false);
    map_memory_.setCostmapUpdated(false);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
