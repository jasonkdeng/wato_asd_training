#include <chrono>
#include <memory>

#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

MapMemoryNode::MapMemoryNode() : Node("map_memory_node") {
  mapMemory_ = std::make_shared<robot::MapMemoryCore>(this->get_logger());

  localMapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::handleLocalMap, this, std::placeholders::_1));

  odomSubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::handleOdometry, this, std::placeholders::_1));

  globalMapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  publishTimer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::publishGlobalMap, this));

  geometry_msgs::msg::Pose mapOrigin;
  mapOrigin.position.x = -25.0;
  mapOrigin.position.y = -25.0;
  mapOrigin.orientation.w = 1.0;

  mapMemory_->initializeMap(0.25, 300, 300, mapOrigin);

  RCLCPP_INFO(this->get_logger(), "Map memory node initialized.");
}

void MapMemoryNode::handleLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty local map.");
    return;
  }

  double distanceMoved =
      std::hypot(robotPositionX_ - previousPositionX_, robotPositionY_ - previousPositionY_);

  previousPositionX_ = robotPositionX_;
  previousPositionY_ = robotPositionY_;

  mapMemory_->integrateLocalMap(msg, robotPositionX_, robotPositionY_, robotOrientationTheta_);
}

void MapMemoryNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robotPositionX_ = msg->pose.pose.position.x;
  robotPositionY_ = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  robotOrientationTheta_ = quaternionToYaw(qx, qy, qz, qw);
}

void MapMemoryNode::publishGlobalMap() {
  auto globalMap = mapMemory_->getGlobalMap();
  globalMap->header.stamp = this->now();
  globalMap->header.frame_id = "sim_world";
  globalMapPublisher_->publish(*globalMap);
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) {
  double sinyCosp = 2.0 * (w * z + x * y);
  double cosyCosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(sinyCosp, cosyCosp);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}