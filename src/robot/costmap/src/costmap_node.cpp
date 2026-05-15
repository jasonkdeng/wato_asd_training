#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  this->declare_parameter<double>("resolution", resolution_);
  this->declare_parameter<int>("width", width_);
  this->declare_parameter<int>("height", height_);
  this->declare_parameter<double>("origin_x", originX_);
  this->declare_parameter<double>("origin_y", originY_);
  this->declare_parameter<int>("inflation_radius", 8);

  resolution_ = this->get_parameter("resolution").as_double();
  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();
  originX_ = this->get_parameter("origin_x").as_double();
  originY_ = this->get_parameter("origin_y").as_double();
  inflationRadius_ = this->get_parameter("inflation_radius").as_int();

  costmap_.configure(resolution_, width_, height_, originX_, originY_, inflationRadius_);

  lidarSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmapPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  RCLCPP_INFO(this->get_logger(), "CostmapNode initialized and subscribing to /lidar");
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.initializeCostmap();

  double angleMin = scan->angle_min;
  double angleIncrement = scan->angle_increment;
  const auto& ranges = scan->ranges;
  size_t rangesSize = ranges.size();

  for (size_t i = 0; i < rangesSize; ++i) {
    double range = ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      double angle = angleMin + i * angleIncrement;
      int xGrid, yGrid;
      costmap_.convertToGrid(range, angle, xGrid, yGrid);
      costmap_.markObstacle(xGrid, yGrid);
    }
  }

  costmap_.inflateObstacles();
  costmap_.enforceHardBoundary();
  costmap_.publishCostmap(costmapPub_, scan);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}