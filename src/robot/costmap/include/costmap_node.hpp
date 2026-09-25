#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                     int width, int height, double resolution);

private:
  robot::CostmapCore costmap_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  void publishCostmap(const std_msgs::msg::Header &scan_header, int width,
                      int height, double resolution);
};

#endif
