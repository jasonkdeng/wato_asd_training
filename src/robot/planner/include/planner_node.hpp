#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void processParameters();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goalMsg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void timerCallback();

    void publishPath();
    void resetGoal();

  private:
    robot::PlannerCore planner_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goalSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Costmap & Mutex
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    std::mutex mapMutex_;

    // Current goal
    geometry_msgs::msg::PointStamped currentGoal_;
    bool activeGoal_ = false;
    rclcpp::Time planStartTime_;

    // Robot odometry
    bool haveOdom_ = false;
    double odomX_ = 0.0;
    double odomY_ = 0.0;

    // Parameters
    std::string mapTopic_;
    std::string goalTopic_;
    std::string odomTopic_;
    std::string pathTopic_;

    double smoothingFactor_;
    int iterations_;
    double goalTolerance_;
    double planTimeout_;
    double goalSearchRadius_;
    int clearanceCostThreshold_;
};

#endif 