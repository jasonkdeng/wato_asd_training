#ifndef COSTMAP_NODE_HPP
#define COSTMAP_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarSub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapPub_;

    robot::CostmapCore costmap_;

    double resolution_ = 0.15;
    int width_ = 300;
    int height_ = 300;
    double originX_ = -25.0;
    double originY_ = -25.0;
    int inflationRadius_ = 8;
};

#endif 