#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    // Read and load in ROS2 parameters
    void processParameters();

    // Utility: Convert quaternion to yaw
    double quaternionToYaw(double x, double y, double z, double w);

    // Callback for path
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // Callback for odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Main loop to continuously follow the path
    void followPath();

    // Timer callback
    void timerCallback();

  private:
    robot::ControlCore control_;

    // Subscriber and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSubscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot state
    double robotX_;
    double robotY_;
    double robotTheta_;

    // ROS2 parameter topics
    std::string pathTopic_;
    std::string odomTopic_;
    std::string cmdVelTopic_;

    // ROS2 parameter values
    int controlPeriodMs_;
    double lookaheadDistance_;
    double steeringGain_;
    double maxSteeringAngle_;
    double linearVelocity_;
};

#endif