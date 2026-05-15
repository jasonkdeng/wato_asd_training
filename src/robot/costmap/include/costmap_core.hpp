#ifndef COSTMAP_CORE_HPP
#define COSTMAP_CORE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
public:
    CostmapCore(const rclcpp::Logger& logger);

    void configure(double resolution, int width, int height,
                   double originX, double originY, int inflationRadius);

    void initializeCostmap();

    void transformToChassisFrame(double &x, double &y);

    void convertToGrid(double range, double angle, int &xGrid, int &yGrid);

    void markObstacle(int xGrid, int yGrid);

    void inflateObstacles();

    void enforceHardBoundary();

    void publishCostmap(
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapPub,
        const sensor_msgs::msg::LaserScan::SharedPtr& laserScanMsg);

private:
    std::vector<int> costmap_;  // Flat 1D array for cache efficiency
    rclcpp::Logger logger_;

    int costmapWidth_ = 300;
    int costmapHeight_ = 300;
    int inflationRadius_ = 8;
    double resolution_ = 0.15;
    double originX_ = -25.0;
    double originY_ = -25.0;
    int lidarOffsetCells_ = 5;  // Offset lidar readings backward (away from vehicle front)
};

}  // namespace robot
#endif  // COSTMAP_CORE_HPP