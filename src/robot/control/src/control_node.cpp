#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "control_node.hpp"

ControlNode::ControlNode()
  : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  processParameters();

  pathSubscriber_ = this->create_subscription<nav_msgs::msg::Path>(
      pathTopic_, 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odomSubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odomTopic_, 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmdVelPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic_, 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(controlPeriodMs_),
      std::bind(&ControlNode::timerCallback, this));

  control_.initControlCore(lookaheadDistance_, maxSteeringAngle_, steeringGain_, linearVelocity_);
}

void ControlNode::processParameters() {
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<int>("control_period_ms", 100);
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("steering_gain", 1.5);
  this->declare_parameter<double>("max_steering_angle", 1.5);
  this->declare_parameter<double>("linear_velocity", 1.5);

  pathTopic_ = this->get_parameter("path_topic").as_string();
  odomTopic_ = this->get_parameter("odom_topic").as_string();
  cmdVelTopic_ = this->get_parameter("cmd_vel_topic").as_string();
  controlPeriodMs_ = this->get_parameter("control_period_ms").as_int();
  lookaheadDistance_ = this->get_parameter("lookahead_distance").as_double();
  steeringGain_ = this->get_parameter("steering_gain").as_double();
  maxSteeringAngle_ = this->get_parameter("max_steering_angle").as_double();
  linearVelocity_ = this->get_parameter("linear_velocity").as_double();
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robotX_ = msg->pose.pose.position.x;
  robotY_ = msg->pose.pose.position.y;

  robotTheta_ = quaternionToYaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
}

void ControlNode::followPath() {
  if (control_.isPathEmpty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "Path is empty. Waiting for new path.");
  }

  geometry_msgs::msg::Twist cmdVel = control_.calculateControlCommand(robotX_, robotY_, robotTheta_);
  cmdVelPublisher_->publish(cmdVel);
}

void ControlNode::timerCallback() {
  followPath();
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w) {
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
