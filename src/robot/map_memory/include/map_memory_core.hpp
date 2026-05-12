#ifndef MAP_MEMORY_CORE_HPP
#define MAP_MEMORY_CORE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace robot {

class MapMemoryCore {
public:
    MapMemoryCore(const rclcpp::Logger& logger);

    void initializeMap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin);

    void integrateLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& localMap,
                           double robotX, double robotY, double robotTheta);

    nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap() const;

private:
    bool mapCoordinatesToGlobal(double worldX, double worldY, int& globalX, int& globalY) const;

    double computeDistance(double x1, double y1, double x2, double y2) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr globalMap_;
    std::vector<bool> updatedCells_;

    double lastRobotX_;
    double lastRobotY_;
    double moveThreshold_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP