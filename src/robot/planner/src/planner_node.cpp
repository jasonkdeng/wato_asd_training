#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Load ROS2 yaml parameters
  processParameters();

  // Subscribers
  mapSub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      mapTopic_, 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goalSub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      goalTopic_, 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odomTopic_, 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  pathPub_ = this->create_publisher<nav_msgs::msg::Path>(pathTopic_, 10);

  // Timer to check goal/timeout status periodically (500 ms)
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                   std::bind(&PlannerNode::timerCallback, this));

  planner_.initPlanner(smoothingFactor_, iterations_, goalTolerance_, goalSearchRadius_,
                       clearanceCostThreshold_);
}

void PlannerNode::processParameters() {
  // Declare and get parameters
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("goal_topic", "/goal_pose");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<double>("smoothing_factor", 0.2);
  this->declare_parameter<int>("iterations", 20);
  this->declare_parameter<double>("goal_tolerance", 0.3);
  this->declare_parameter<double>("plan_timeout_seconds", 10.0);
  this->declare_parameter<double>("goal_search_radius", 1.5);
  this->declare_parameter<int>("clearance_cost_threshold", 60);

  mapTopic_ = this->get_parameter("map_topic").as_string();
  goalTopic_ = this->get_parameter("goal_topic").as_string();
  odomTopic_ = this->get_parameter("odom_topic").as_string();
  pathTopic_ = this->get_parameter("path_topic").as_string();
  smoothingFactor_ = this->get_parameter("smoothing_factor").as_double();
  iterations_ = this->get_parameter("iterations").as_int();
  goalTolerance_ = this->get_parameter("goal_tolerance").as_double();
  planTimeout_ = this->get_parameter("plan_timeout_seconds").as_double();
  goalSearchRadius_ = this->get_parameter("goal_search_radius").as_double();
  clearanceCostThreshold_ = this->get_parameter("clearance_cost_threshold").as_int();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) {
  {
    std::lock_guard<std::mutex> lock(mapMutex_);
    map_ = mapMsg;
  }

  // If we have an active goal, re-run plan
  if (activeGoal_) {
    double elapsed = (now() - planStartTime_).seconds();
    if (elapsed <= planTimeout_) {
      RCLCPP_INFO(this->get_logger(),
                  "Map updated => Replanning for current goal (time elapsed: %.2f).", elapsed);
      publishPath();
    }
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goalMsg) {
  if (activeGoal_) {
    RCLCPP_WARN(this->get_logger(), "Ignoring new goal; a goal is already active.");
    return;
  }

  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "No costmap available yet. Cannot set goal.");
    return;
  }

  currentGoal_ = *goalMsg;
  activeGoal_ = true;
  planStartTime_ = now();

  RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f)", goalMsg->point.x,
              goalMsg->point.y);

  publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
  odomX_ = odomMsg->pose.pose.position.x;
  odomY_ = odomMsg->pose.pose.position.y;
  haveOdom_ = true;
}

void PlannerNode::timerCallback() {
  if (!activeGoal_) {
    return;
  }

  // Check if we've timed out
  double elapsed = (now() - planStartTime_).seconds();
  if (elapsed > planTimeout_) {
    RCLCPP_WARN(this->get_logger(), "Plan timed out after %.2f seconds. Resetting goal.",
                elapsed);
    resetGoal();
    return;
  }

  // Check if we reached the goal (use hypot for efficiency)
  double distance = std::hypot(odomX_ - currentGoal_.point.x, odomY_ - currentGoal_.point.y);
  if (distance < goalTolerance_) {
    RCLCPP_WARN(this->get_logger(), "Plan succeeded! Elapsed Time: %.2f", elapsed);
    resetGoal();
    return;
  }
}

void PlannerNode::publishPath() {
  if (!haveOdom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet. Cannot plan.");
    resetGoal();
    return;
  }

  double startWorldX = odomX_;
  double startWorldY = odomY_;

  {
    std::lock_guard<std::mutex> lock(mapMutex_);
    if (!planner_.planPath(startWorldX, startWorldY, currentGoal_.point.x, currentGoal_.point.y,
                           map_)) {
      RCLCPP_ERROR(this->get_logger(), "Plan Failed.");
      resetGoal();
      return;
    }
  }

  nav_msgs::msg::Path pathMsg = *planner_.getPath();
  pathMsg.header.stamp = this->now();
  pathMsg.header.frame_id = map_->header.frame_id;

  pathPub_->publish(pathMsg);
}

void PlannerNode::resetGoal() {
  activeGoal_ = false;
  RCLCPP_INFO(this->get_logger(), "Resetting active goal.");

  // Publish an empty path
  nav_msgs::msg::Path emptyPath;
  emptyPath.header.stamp = this->now();

  {
    std::lock_guard<std::mutex> lock(mapMutex_);
    if (map_) {
      emptyPath.header.frame_id = map_->header.frame_id;
    } else {
      emptyPath.header.frame_id = "sim_world";
    }
  }

  pathPub_->publish(emptyPath);

  RCLCPP_INFO(this->get_logger(), "Published empty path to stop the robot.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}