#ifndef ODOMETRY_SPOOF_NODE_HPP_
#define ODOMETRY_SPOOF_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"

class OdometrySpoofNode : public rclcpp::Node {
  public:
    OdometrySpoofNode();

  private:
    void timerCallback();

    // Odom Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

    // Transform Utilities
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    // Timer to periodically lookup transforms
    rclcpp::TimerBase::SharedPtr timer_;

    // Check if last transform was found
    bool hasLastTransform_ = false;

    // Variables to store previous transform
    rclcpp::Time lastTime_;
    tf2::Vector3 lastPosition_;
    tf2::Quaternion lastOrientation_;
};

#endif 