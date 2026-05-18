#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::configure(double resolution, int width, int height,
                            double originX, double originY, int inflationRadius) {
    resolution_ = resolution;
    costmapWidth_ = width;
    costmapHeight_ = height;
    originX_ = originX;
    originY_ = originY;
    inflationRadius_ = std::max(0, inflationRadius);
    costmap_.assign(costmapWidth_ * costmapHeight_, 0);
}

void CostmapCore::initializeCostmap() {
    costmap_.assign(costmapWidth_ * costmapHeight_, 0);
}

void CostmapCore::transformToChassisFrame(double &x, double &y) {
    double robotX = 0.0;
    double robotY = 0.0;
    double robotTheta = 0.0;

    x -= robotX;
    y -= robotY;

    double cosTheta = cos(robotTheta);
    double sinTheta = sin(robotTheta);

    double xNew = x * cosTheta + y * sinTheta;
    double yNew = -x * sinTheta + y * cosTheta;

    x = xNew;
    y = yNew;
}

void CostmapCore::convertToGrid(double range, double angle, int &xGrid, int &yGrid) {
    double x = range * cos(angle);
    double y = range * sin(angle);

    transformToChassisFrame(x, y);
    xGrid = (x - originX_) / resolution_;
    yGrid = (y - originY_) / resolution_;
    
    // Offset lidar readings backward (negative x) by lidarOffsetCells_ to account for lidar mounting position
    xGrid -= lidarOffsetCells_;
}

void CostmapCore::markObstacle(int xGrid, int yGrid) {
    if (xGrid >= 0 && xGrid < costmapWidth_ && yGrid >= 0 && yGrid < costmapHeight_) {
        costmap_[yGrid * costmapWidth_ + xGrid] = 100;
    }
}

void CostmapCore::dilateObstacles() {
    std::vector<int> dilated = costmap_;

    for (int y = 1; y < costmapHeight_ - 1; ++y) {
        for (int x = 1; x < costmapWidth_ - 1; ++x) {
            if (costmap_[y * costmapWidth_ + x] == 100) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        dilated[(y + dy) * costmapWidth_ + (x + dx)] = 100;
                    }
                }
            }
        }
    }

    costmap_ = dilated;
}

void CostmapCore::inflateObstacles() {
    int maxCost = 100;
    int radiusSq = inflationRadius_ * inflationRadius_;
    
    for (int y = 0; y < costmapHeight_; ++y) {
        for (int x = 0; x < costmapWidth_; ++x) {
            if (costmap_[y * costmapWidth_ + x] == maxCost) {
                for (int dy = -inflationRadius_; dy <= inflationRadius_; ++dy) {
                    for (int dx = -inflationRadius_; dx <= inflationRadius_; ++dx) {
                        int distSq = dx * dx + dy * dy;

                        if (distSq <= radiusSq) {
                            int inflateX = x + dx;
                            int inflateY = y + dy;

                            if (inflateX >= 0 && inflateX < costmapWidth_ &&
                                inflateY >= 0 && inflateY < costmapHeight_) {
                                double dist = std::sqrt(distSq);
                                double decay = std::exp(-3.0 * dist / inflationRadius_);
                                int inflatedCost = static_cast<int>(maxCost * decay);
                                int flatIndex = inflateY * costmapWidth_ + inflateX;
                                costmap_[flatIndex] = std::max(costmap_[flatIndex], inflatedCost);
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostmapCore::enforceHardBoundary() {
    int maxCost = 100;
    int boundaryCost = 90;

    for (int y = 0; y < costmapHeight_; ++y) {
        for (int x = 0; x < costmapWidth_; ++x) {
            int index = y * costmapWidth_ + x;
            if (costmap_[index] != maxCost) {
                continue;
            }

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }

                    int neighborX = x + dx;
                    int neighborY = y + dy;
                    if (neighborX < 0 || neighborX >= costmapWidth_ ||
                        neighborY < 0 || neighborY >= costmapHeight_) {
                        continue;
                    }

                    int neighborIndex = neighborY * costmapWidth_ + neighborX;
                    costmap_[neighborIndex] = std::max(costmap_[neighborIndex], boundaryCost);
                }
            }
        }
    }
}

void CostmapCore::publishCostmap(
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapPub,
    const sensor_msgs::msg::LaserScan::SharedPtr& laserScanMsg) {

    auto occupancyGridMsg = nav_msgs::msg::OccupancyGrid();

    occupancyGridMsg.header.stamp = rclcpp::Clock().now();
    occupancyGridMsg.header = laserScanMsg->header;

    occupancyGridMsg.info.resolution = resolution_;
    occupancyGridMsg.info.width = costmapWidth_;
    occupancyGridMsg.info.height = costmapHeight_;

    occupancyGridMsg.info.origin.position.x = originX_;
    occupancyGridMsg.info.origin.position.y = originY_;

    occupancyGridMsg.info.origin.orientation.x = 0.0;
    occupancyGridMsg.info.origin.orientation.y = 0.0;
    occupancyGridMsg.info.origin.orientation.z = 0.0;
    occupancyGridMsg.info.origin.orientation.w = 1.0;

    occupancyGridMsg.data.resize(costmapWidth_ * costmapHeight_);

    for (int y = 0; y < costmapHeight_; ++y) {
        for (int x = 0; x < costmapWidth_; ++x) {
            occupancyGridMsg.data[y * costmapWidth_ + x] = costmap_[y * costmapWidth_ + x];
        }
    }
    costmapPub->publish(occupancyGridMsg);
}

}