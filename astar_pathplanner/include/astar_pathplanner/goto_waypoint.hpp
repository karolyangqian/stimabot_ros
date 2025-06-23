#ifndef ASTAR_PATHPLANNER__GOTO_WAYPOINT_HPP_
#define ASTAR_PATHPLANNER__GOTO_WAYPOINT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

namespace astar_pathplanner
{

class GotoWaypoint : public rclcpp::Node
{
public:
  GotoWaypoint();
  ~GotoWaypoint() = default;

private:
  // ROS2 Publishers and Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // Timer for motion control
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Motion parameters
  double forward_speed_;
  double rotational_speed_;
  
  // Current state
  nav_msgs::msg::Odometry current_odom_;  // Store full odometry message
  geometry_msgs::msg::Pose goal_pose_;
  std::string goal_frame_;  // Track the frame of the goal
  std::vector<geometry_msgs::msg::PoseStamped> current_path_;  // Current path to follow
  size_t current_waypoint_index_;  // Index of current waypoint in path
  bool has_odom_;
  bool has_goal_pose_;
  bool has_path_;
  bool is_moving_;
  
  // Motion control state
  enum class MotionState {
    IDLE,
    ROTATING_TO_GOAL,
    MOVING_TO_GOAL,
    FOLLOWING_PATH,
    COMPLETED
  };
  
  MotionState motion_state_;
  rclcpp::Time motion_start_time_;
  double target_rotation_duration_;
  double target_translation_duration_;
  
  // Callback functions
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void controlTimerCallback();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  // Helper functions
  double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);
  double calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);
  double calculateAngleDifference(double current_yaw, double target_yaw);
  double calculateTargetYaw(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to);
  void startMotion();
  void startPathFollowing();
  void stopMotion();
  void publishVelocity(double linear_x, double angular_z);
  bool isCloseToWaypoint(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target, double tolerance = 0.1);
  void moveToNextWaypoint();
  
  // Utility functions
  double normalizeAngle(double angle);
};

}  // namespace astar_pathplanner

#endif  // ASTAR_PATHPLANNER__GOTO_WAYPOINT_HPP_