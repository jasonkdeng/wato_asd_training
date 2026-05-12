#include "map_memory_core.hpp"
#include <cmath>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger), lastRobotX_(0), lastRobotY_(0), moveThreshold_(5.0) {
  globalMap_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

void MapMemoryCore::initializeMap(double resolution, int width, int height,
                                  const geometry_msgs::msg::Pose& origin) {
  globalMap_->info.resolution = resolution;
  globalMap_->info.width = width;
  globalMap_->info.height = height;
  globalMap_->info.origin = origin;
  globalMap_->data.resize(width * height, 0);

  updatedCells_.resize(width * height, false);

  lastRobotX_ = origin.position.x;
  lastRobotY_ = origin.position.y;

  RCLCPP_INFO(logger_, "Initialized global map");
}

void MapMemoryCore::integrateLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& localMap,
                                       double robotX, double robotY, double robotTheta) {
  double distanceMoved = computeDistance(lastRobotX_, lastRobotY_, robotX, robotY);

  if (distanceMoved >= moveThreshold_) {
    for (unsigned int y = 0; y < localMap->info.height; ++y) {
      for (unsigned int x = 0; x < localMap->info.width; ++x) {
        int index = y * localMap->info.width + x;
        if (localMap->data[index] < 0) continue;

        double localX = localMap->info.origin.position.x + x * localMap->info.resolution;
        double localY = localMap->info.origin.position.y + y * localMap->info.resolution;

        double worldX = robotX + localX * std::cos(robotTheta) - localY * std::sin(robotTheta);
        double worldY = robotY + localX * std::sin(robotTheta) + localY * std::cos(robotTheta);

        int globalX, globalY;
        if (mapCoordinatesToGlobal(worldX, worldY, globalX, globalY)) {
          int globalIndex = globalY * globalMap_->info.width + globalX;

          if (!updatedCells_[globalIndex]) {
            globalMap_->data[globalIndex] = localMap->data[index];
            updatedCells_[globalIndex] = true;
          } else {
            globalMap_->data[globalIndex] =
                std::max(globalMap_->data[globalIndex], localMap->data[index]);
          }
        }
      }
    }

    // Update the robot position
    lastRobotX_ = robotX;
    lastRobotY_ = robotY;
  }
}

double MapMemoryCore::computeDistance(double x1, double y1, double x2, double y2) const {
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

bool MapMemoryCore::mapCoordinatesToGlobal(double worldX, double worldY, int& globalX,
                                            int& globalY) const {
  double originX = globalMap_->info.origin.position.x;
  double originY = globalMap_->info.origin.position.y;
  double resolution = globalMap_->info.resolution;

  if (worldX < originX || worldY < originY) return false;

  globalX = static_cast<int>((worldX - originX) / resolution);
  globalY = static_cast<int>((worldY - originY) / resolution);

  if (globalX < 0 || globalX >= static_cast<int>(globalMap_->info.width) ||
      globalY < 0 || globalY >= static_cast<int>(globalMap_->info.height)) {
    return false;
  }

  return true;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getGlobalMap() const {
  return globalMap_;
}

}