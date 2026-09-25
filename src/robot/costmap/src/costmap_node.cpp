#include <algorithm>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 400);
  this->declare_parameter<int>("height", 400);
  this->declare_parameter<double>("inflation_radius", 0.5);
  this->declare_parameter<int>("max_cost", 100);

  double resolution = this->get_parameter("resolution").as_double();
  int width = this->get_parameter("width").as_int();
  int height = this->get_parameter("height").as_int();
  double inflation_radius = this->get_parameter("inflation_radius").as_double();
  int max_cost = this->get_parameter("max_cost").as_int();

  costmap_.initialize(width, height, resolution, inflation_radius, max_cost);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      [this, width, height,
       resolution](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        CostmapNode::laserCallback(scan, width, height, resolution);
      });
  grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan, int width, int height,
    double resolution) {

  this->costmap_.processScan(scan->angle_min, scan->angle_increment,
                             scan->ranges, scan->range_min, scan->range_max);

  publishCostmap(scan->header, width, height, resolution);
}

void CostmapNode::publishCostmap(const std_msgs::msg::Header &scan_header,
                                 int width, int height, double resolution) {
  const auto &grid = this->costmap_.getGrid();
  auto occupancy_grid = nav_msgs::msg::OccupancyGrid();
  occupancy_grid.header = scan_header;
  occupancy_grid.info.height = height;
  occupancy_grid.info.width = width;
  occupancy_grid.info.resolution = resolution;
  occupancy_grid.info.origin.position.x = this->costmap_.getOriginX();
  occupancy_grid.info.origin.position.y = this->costmap_.getOriginY();
  occupancy_grid.data.reserve(width * height);

  for (const auto &row : grid) {
    for (const auto &val : row) {
      occupancy_grid.data.emplace_back(std::clamp(val, -1, 100));
    }
  }

  grid_pub_->publish(occupancy_grid);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
