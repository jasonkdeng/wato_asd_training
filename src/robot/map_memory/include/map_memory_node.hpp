#ifndef MAP_MEMORY_NODE_HPP
#define MAP_MEMORY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    void handleLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishGlobalMap();
    double quaternionToYaw(double x, double y, double z, double w);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr localMapSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr globalMapPublisher_;
    rclcpp::TimerBase::SharedPtr publishTimer_;

    std::shared_ptr<robot::MapMemoryCore> mapMemory_;

    double robotPositionX_ = 0.0;
    double robotPositionY_ = 0.0;
    double robotOrientationTheta_ = 0.0;
};

#endif