#ifndef ASTAR_PATHPLANNER_HPP_
#define ASTAR_PATHPLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "astar_pathplanner/astar.hpp"

namespace astar_pathplanner
{

class AStarPathplanner : public rclcpp::Node
{
public:
  AStarPathplanner();
  ~AStarPathplanner() = default;

private:
  // ROS2 Publishers and Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  // Current state
  nav_msgs::msg::Odometry current_odom_;
  bool has_odom_;
  bool has_map_;
  
  // A* algorithm instance
  AStar astar_;

  // Callback functions
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};

}  // namespace astar_pathplanner

#endif  // ASTAR_PATHPLANNER_HPP_