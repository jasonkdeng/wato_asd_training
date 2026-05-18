#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    void initControlCore(
        double lookaheadDistance,
        double maxSteeringAngle,
        double steeringGain,
        double linearVelocity);

    void updatePath(nav_msgs::msg::Path path);

    bool isPathEmpty();

    unsigned int findLookaheadPoint(
        double robotX, double robotY, double robotTheta);

    geometry_msgs::msg::Twist calculateControlCommand(
        double robotX, double robotY, double robotTheta);

  private:
    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double lookaheadDistance_;
    double maxSteeringAngle_;
    double steeringGain_;
    double linearVelocity_;
    size_t lastLookaheadIndex_ = 0;
};

}
#endif  // CONTROL_CORE_HPP_