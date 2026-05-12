#include "control_core.hpp"
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : path_(nav_msgs::msg::Path()), logger_(logger) {}

void ControlCore::initControlCore(
  double lookaheadDistance,
  double maxSteeringAngle,
  double steeringGain,
  double linearVelocity) {
  lookaheadDistance_ = lookaheadDistance;
  maxSteeringAngle_ = maxSteeringAngle;
  steeringGain_ = steeringGain;
  linearVelocity_ = linearVelocity;
}

void ControlCore::updatePath(nav_msgs::msg::Path path) {
  RCLCPP_INFO(logger_, "Path Updated");
  path_ = path;
}

bool ControlCore::isPathEmpty() {
  return path_.poses.empty();
}

geometry_msgs::msg::Twist ControlCore::calculateControlCommand(
  double robotX, double robotY, double robotTheta) {
  geometry_msgs::msg::Twist twist;

  unsigned int lookaheadIndex = findLookaheadPoint(robotX, robotY, robotTheta);

  if (lookaheadIndex >= path_.poses.size()) {
    return twist;
  }

  double lookaheadX = path_.poses[lookaheadIndex].pose.position.x;
  double lookaheadY = path_.poses[lookaheadIndex].pose.position.y;

  double dx = lookaheadX - robotX;
  double dy = lookaheadY - robotY;

  double angleToLookahead = std::atan2(dy, dx);
  double steeringAngle = angleToLookahead - robotTheta;

  // Normalize steering angle to [-π, π]
  if (steeringAngle > M_PI) {
    steeringAngle -= 2 * M_PI;
  } else if (steeringAngle < -M_PI) {
    steeringAngle += 2 * M_PI;
  }

  if (std::abs(steeringAngle) > std::abs(maxSteeringAngle_)) {
    twist.linear.x = 0;
  } else {
    twist.linear.x = linearVelocity_;
  }

  steeringAngle = std::max(-maxSteeringAngle_, std::min(steeringAngle, maxSteeringAngle_));
  twist.angular.z = steeringAngle * steeringGain_;

  return twist;
}

unsigned int ControlCore::findLookaheadPoint(
  double robotX, double robotY, double robotTheta) {
  double minDistance = std::numeric_limits<double>::max();
  int lookaheadIndex = 0;
  bool foundForward = false;
  size_t pathSize = path_.poses.size();

  // Search for closest forward-facing point
  for (size_t i = 0; i < pathSize; ++i) {
    double dx = path_.poses[i].pose.position.x - robotX;
    double dy = path_.poses[i].pose.position.y - robotY;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < lookaheadDistance_) {
      continue;
    }

    double angleToPoint = std::atan2(dy, dx);
    double angleDiff = angleToPoint - robotTheta;

    // Normalize angle difference to [-π, π]
    if (angleDiff > M_PI) {
      angleDiff -= 2 * M_PI;
    } else if (angleDiff < -M_PI) {
      angleDiff += 2 * M_PI;
    }

    if (std::abs(angleDiff) < M_PI / 2) {
      if (distance < minDistance) {
        minDistance = distance;
        lookaheadIndex = i;
        foundForward = true;
      }
    }
  }

  // If no forward point found, find closest point regardless of direction
  if (!foundForward) {
    minDistance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < pathSize; ++i) {
      double dx = path_.poses[i].pose.position.x - robotX;
      double dy = path_.poses[i].pose.position.y - robotY;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < lookaheadDistance_) {
        continue;
      }

      if (distance < minDistance) {
        minDistance = distance;
        lookaheadIndex = i;
      }
    }
  }

  return lookaheadIndex;
}

}  
