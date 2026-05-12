#include <chrono>
#include <memory>

#include "odometry_spoof.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

OdometrySpoofNode::OdometrySpoofNode() : Node("odometry_spoof") {
  // Create publisher for Odometry messages
  odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/filtered", 10);

  // Create a TF buffer and listener
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Create a timer to fetch transform & publish odometry at ~10 Hz
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&OdometrySpoofNode::timerCallback, this));
}

void OdometrySpoofNode::timerCallback() {
  // Look up the transform from sim_world -> robot/chassis/lidar
  const std::string targetFrame = "robot/chassis/lidar";
  const std::string sourceFrame = "sim_world";

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_->lookupTransform(
        sourceFrame,
        targetFrame,
        tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not transform %s to %s: %s",
                         sourceFrame.c_str(), targetFrame.c_str(), ex.what());
    return;
  }

  // Create an Odometry message
  nav_msgs::msg::Odometry odomMsg;

  // Fill header
  odomMsg.header.stamp = transformStamped.header.stamp;
  odomMsg.header.frame_id = sourceFrame;
  odomMsg.child_frame_id = targetFrame;

  // Pose from TF
  odomMsg.pose.pose.position.x = transformStamped.transform.translation.x;
  odomMsg.pose.pose.position.y = transformStamped.transform.translation.y;
  odomMsg.pose.pose.position.z = transformStamped.transform.translation.z;
  odomMsg.pose.pose.orientation = transformStamped.transform.rotation;

  // Compute twist from difference in transforms
  if (hasLastTransform_) {
    rclcpp::Time currentTime = transformStamped.header.stamp;
    double dt = (currentTime - lastTime_).seconds();

    if (dt > 0.0) {
      // Linear velocity
      double dx = odomMsg.pose.pose.position.x - lastPosition_.x();
      double dy = odomMsg.pose.pose.position.y - lastPosition_.y();
      double dz = odomMsg.pose.pose.position.z - lastPosition_.z();

      odomMsg.twist.twist.linear.x = dx / dt;
      odomMsg.twist.twist.linear.y = dy / dt;
      odomMsg.twist.twist.linear.z = dz / dt;

      // Angular velocity
      tf2::Quaternion qLast(lastOrientation_.x(),
                             lastOrientation_.y(),
                             lastOrientation_.z(),
                             lastOrientation_.w());

      tf2::Quaternion qCurrent(transformStamped.transform.rotation.x,
                               transformStamped.transform.rotation.y,
                               transformStamped.transform.rotation.z,
                               transformStamped.transform.rotation.w);

      // Orientation difference: q_diff = q_last.inverse() * q_current
      tf2::Quaternion qDiff = qLast.inverse() * qCurrent;

      double rollDiff, pitchDiff, yawDiff;
      tf2::Matrix3x3(qDiff).getRPY(rollDiff, pitchDiff, yawDiff);

      // Angular velocity (rad/s)
      odomMsg.twist.twist.angular.x = rollDiff / dt;
      odomMsg.twist.twist.angular.y = pitchDiff / dt;
      odomMsg.twist.twist.angular.z = yawDiff / dt;
    } else {
      // If dt == 0, set velocity to zero
      odomMsg.twist.twist.linear.x = 0.0;
      odomMsg.twist.twist.linear.y = 0.0;
      odomMsg.twist.twist.linear.z = 0.0;
      odomMsg.twist.twist.angular.x = 0.0;
      odomMsg.twist.twist.angular.y = 0.0;
      odomMsg.twist.twist.angular.z = 0.0;
    }
  } else {
    // No previous transform yet; set velocity to zero
    odomMsg.twist.twist.linear.x = 0.0;
    odomMsg.twist.twist.linear.y = 0.0;
    odomMsg.twist.twist.linear.z = 0.0;
    odomMsg.twist.twist.angular.x = 0.0;
    odomMsg.twist.twist.angular.y = 0.0;
    odomMsg.twist.twist.angular.z = 0.0;

    // Mark that we've now received at least one transform
    hasLastTransform_ = true;
  }

  // Publish odom
  odomPub_->publish(odomMsg);

  // Store current transform as "last" for next iteration
  lastTime_ = transformStamped.header.stamp;
  lastPosition_.setValue(
      odomMsg.pose.pose.position.x,
      odomMsg.pose.pose.position.y,
      odomMsg.pose.pose.position.z);
  lastOrientation_.setValue(
      odomMsg.pose.pose.orientation.x,
      odomMsg.pose.pose.orientation.y,
      odomMsg.pose.pose.orientation.z,
      odomMsg.pose.pose.orientation.w);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySpoofNode>());
  rclcpp::shutdown();
  return 0;
}