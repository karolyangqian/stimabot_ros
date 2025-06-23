//  Copyright 2025 Karol Yangqian Poetracahya

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.


#include "astar_pathplanner/goto_waypoint.hpp"

namespace astar_pathplanner
{

GotoWaypoint::GotoWaypoint() : Node("goto_waypoint")
{
  // Declare parameters
  this->declare_parameter("forward_speed", 0.2);
  this->declare_parameter("rotational_speed", 0.5);
  
  // Get parameters
  this->get_parameter("forward_speed", forward_speed_);
  this->get_parameter("rotational_speed", rotational_speed_);
  
  RCLCPP_INFO(this->get_logger(), "Goto Waypoint Node initialized");
  RCLCPP_INFO(this->get_logger(), "Forward speed: %.2f m/s", forward_speed_);
  RCLCPP_INFO(this->get_logger(), "Rotational speed: %.2f rad/s", rotational_speed_);
  
  // Initialize state
  has_odom_ = false;
  has_goal_pose_ = false;
  has_path_ = false;
  is_moving_ = false;
  motion_state_ = MotionState::IDLE;
  current_waypoint_index_ = 0;
  
  // Create subscribers
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&GotoWaypoint::goalPoseCallback, this, std::placeholders::_1));

  plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", 10,
    std::bind(&GotoWaypoint::planCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&GotoWaypoint::odomCallback, this, std::placeholders::_1));
  
  // Create publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  // Create control timer (50 Hz)
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&GotoWaypoint::controlTimerCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "Waiting for odometry, goal pose, or path...");
}

void GotoWaypoint::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
  has_odom_ = true;
}

void GotoWaypoint::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!has_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry data available. Waiting for odometry...");
    return;
  }
  
  goal_pose_ = msg->pose;
  goal_frame_ = msg->header.frame_id;  // Extract frame from goal message
  has_goal_pose_ = true;
  
  double goal_yaw = calculateYawFromQuaternion(goal_pose_.orientation);
  
  RCLCPP_INFO(this->get_logger(), 
    "Goal pose received in frame '%s': (%.2f, %.2f, %.2f°)", 
    goal_frame_.c_str(),
    goal_pose_.position.x, 
    goal_pose_.position.y,
    goal_yaw * 180.0 / M_PI);
  
  // Start motion planning and execution
  startMotion();
}

void GotoWaypoint::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!has_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry data available. Waiting for odometry...");
    return;
  }
  
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path. Ignoring.");
    return;
  }
  
  current_path_ = msg->poses;
  has_path_ = true;
  current_waypoint_index_ = 0;
  
  RCLCPP_INFO(this->get_logger(), 
    "Path received with %zu waypoints in frame '%s'", 
    current_path_.size(),
    msg->header.frame_id.c_str());
  
  // Log first and last waypoints
  const auto& first_pose = current_path_[0].pose;
  const auto& last_pose = current_path_.back().pose;
  
  RCLCPP_INFO(this->get_logger(), 
    "First waypoint: (%.2f, %.2f, %.2f°)", 
    first_pose.position.x, 
    first_pose.position.y,
    calculateYawFromQuaternion(first_pose.orientation) * 180.0 / M_PI);
    
  RCLCPP_INFO(this->get_logger(), 
    "Last waypoint: (%.2f, %.2f, %.2f°)", 
    last_pose.position.x, 
    last_pose.position.y,
    calculateYawFromQuaternion(last_pose.orientation) * 180.0 / M_PI);
  
  // Start path following
  startPathFollowing();
}

void GotoWaypoint::startMotion()
{
  if (!has_odom_ || !has_goal_pose_) {
    return;
  }
  
  // Get current pose from odometry
  const auto& current_pose = current_odom_.pose.pose;
  double distance, target_yaw, current_yaw, angle_diff;
  
  if (goal_frame_ == "base_link") {
    // Goal is relative to current robot position
    distance = sqrt(goal_pose_.position.x * goal_pose_.position.x + 
                   goal_pose_.position.y * goal_pose_.position.y);
    
    target_yaw = atan2(goal_pose_.position.y, goal_pose_.position.x);
    current_yaw = 0.0; // Robot is at origin in its own frame
    angle_diff = target_yaw;
    
    RCLCPP_INFO(this->get_logger(), "Using relative goal in base_link frame");
  } else {
    // Goal is in global coordinates (odom frame)
    distance = calculateDistance(current_pose, goal_pose_);
    target_yaw = calculateTargetYaw(current_pose, goal_pose_);
    current_yaw = calculateYawFromQuaternion(current_pose.orientation);
    angle_diff = calculateAngleDifference(current_yaw, target_yaw);
    
    RCLCPP_INFO(this->get_logger(), "Using global goal in odom frame");
  }
  
  // Calculate durations
  target_rotation_duration_ = std::abs(angle_diff) / rotational_speed_;
  target_translation_duration_ = distance / forward_speed_;

  RCLCPP_INFO(this->get_logger(), 
    "Current pose: (%.2f, %.2f, %.2f°)", 
    current_pose.position.x, 
    current_pose.position.y,
    current_yaw * 180.0 / M_PI);

  RCLCPP_INFO(this->get_logger(), 
    "Goal pose: (%.2f, %.2f, %.2f°)", 
    goal_pose_.position.x, 
    goal_pose_.position.y,
    target_yaw * 180.0 / M_PI);
  
  RCLCPP_INFO(this->get_logger(), 
    "Motion plan: Rotate %.2f° (%.2fs), then move %.2fm (%.2fs)",
    angle_diff * 180.0 / M_PI, target_rotation_duration_,
    distance, target_translation_duration_);
  
  // Start rotation phase
  motion_state_ = MotionState::ROTATING_TO_GOAL;
  motion_start_time_ = this->now();
  is_moving_ = true;
}

void GotoWaypoint::controlTimerCallback()
{
  if (!is_moving_ || !has_odom_) {
    return;
  }
  
  const auto& current_pose = current_odom_.pose.pose;
  
  switch (motion_state_) {
    case MotionState::FOLLOWING_PATH:
      {
        // Check if we're close to the current waypoint
        if (isCloseToWaypoint(current_pose, goal_pose_, 0.15)) {
          // Move to next waypoint
          moveToNextWaypoint();
          return;
        }
        
        // Calculate direction to current waypoint
        double target_yaw = calculateTargetYaw(current_pose, goal_pose_);
        double current_yaw = calculateYawFromQuaternion(current_pose.orientation);
        double angle_diff = calculateAngleDifference(current_yaw, target_yaw);
        
        // Simple proportional control
        double angular_vel = 0.0;
        double linear_vel = 0.0;
        
        // If angle difference is significant, prioritize rotation
        if (std::abs(angle_diff) > 0.1) { // ~6 degrees
          angular_vel = std::copysign(rotational_speed_ * 0.8, angle_diff);
          linear_vel = forward_speed_ * 0.3; // Slow forward while turning
        } else {
          // Move forward when aligned
          linear_vel = forward_speed_;
          angular_vel = angle_diff * 1.0; // Small correction
        }
        
        publishVelocity(linear_vel, angular_vel);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Following path: waypoint %zu/%zu, distance: %.2fm, angle: %.1f°",
          current_waypoint_index_ + 1, current_path_.size(),
          calculateDistance(current_pose, goal_pose_),
          angle_diff * 180.0 / M_PI);
      }
      break;
      
    case MotionState::ROTATING_TO_GOAL:
      {
        double elapsed_time = (this->now() - motion_start_time_).seconds();
        if (elapsed_time < target_rotation_duration_) {
          // Still rotating
          double target_yaw, current_yaw, angle_diff;
          
          if (goal_frame_ == "base_link") {
            // For relative goals, target yaw is relative to start orientation
            target_yaw = atan2(goal_pose_.position.y, goal_pose_.position.x);
            current_yaw = 0.0; // In relative mode, we consider start as 0
            angle_diff = target_yaw;
          } else {
            // For global goals, calculate based on current position
            target_yaw = calculateTargetYaw(current_pose, goal_pose_);
            current_yaw = calculateYawFromQuaternion(current_pose.orientation);
            angle_diff = calculateAngleDifference(current_yaw, target_yaw);
          }
          
          double angular_vel = (angle_diff > 0) ? rotational_speed_ : -rotational_speed_;
          publishVelocity(0.0, angular_vel);
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Rotating... %.1fs remaining", target_rotation_duration_ - elapsed_time);
        } else {
          // Rotation complete, start translation
          publishVelocity(0.0, 0.0);  // Stop rotation
          motion_state_ = MotionState::MOVING_TO_GOAL;
          motion_start_time_ = this->now();
          
          RCLCPP_INFO(this->get_logger(), "Rotation complete. Starting translation.");
        }
      }
      break;
      
    case MotionState::MOVING_TO_GOAL:
      {
        double elapsed_time = (this->now() - motion_start_time_).seconds();
        if (elapsed_time < target_translation_duration_) {
          // Still moving forward
          publishVelocity(forward_speed_, 0.0);
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Moving forward... %.1fs remaining", target_translation_duration_ - elapsed_time);
        } else {
          // Translation complete
          stopMotion();
          motion_state_ = MotionState::COMPLETED;
          
          RCLCPP_INFO(this->get_logger(), "Goal reached! Motion completed.");
        }
      }
      break;
      
    case MotionState::COMPLETED:
    case MotionState::IDLE:
    default:
      // Do nothing
      break;
  }
}

void GotoWaypoint::stopMotion()
{
  publishVelocity(0.0, 0.0);
  is_moving_ = false;
  motion_state_ = MotionState::IDLE;
}

void GotoWaypoint::publishVelocity(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_x;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = angular_z;
  
  cmd_vel_pub_->publish(cmd_vel);
}

double GotoWaypoint::calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
{
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 mat(tf_quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

double GotoWaypoint::calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
{
  double dx = pose2.position.x - pose1.position.x;
  double dy = pose2.position.y - pose1.position.y;
  return sqrt(dx * dx + dy * dy);
}

double GotoWaypoint::calculateAngleDifference(double current_yaw, double target_yaw)
{
  double diff = target_yaw - current_yaw;
  return normalizeAngle(diff);
}

double GotoWaypoint::calculateTargetYaw(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to)
{
  double dx = to.position.x - from.position.x;
  double dy = to.position.y - from.position.y;
  return atan2(dy, dx);
}

double GotoWaypoint::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void GotoWaypoint::startPathFollowing()
{
  if (!has_odom_ || !has_path_ || current_path_.empty()) {
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Starting path following with %zu waypoints", current_path_.size());
  
  // Set the first waypoint as current goal
  current_waypoint_index_ = 0;
  goal_pose_ = current_path_[current_waypoint_index_].pose;
  goal_frame_ = current_path_[current_waypoint_index_].header.frame_id;
  
  // Start motion to first waypoint
  motion_state_ = MotionState::FOLLOWING_PATH;
  is_moving_ = true;
  
  // Calculate motion for first waypoint
  const auto& current_pose = current_odom_.pose.pose;
  double distance = calculateDistance(current_pose, goal_pose_);
  double target_yaw = calculateTargetYaw(current_pose, goal_pose_);
  double current_yaw = calculateYawFromQuaternion(current_pose.orientation);
  double angle_diff = calculateAngleDifference(current_yaw, target_yaw);
  
  RCLCPP_INFO(this->get_logger(), 
    "Moving to waypoint %zu: (%.2f, %.2f) - distance: %.2fm, angle: %.1f°",
    current_waypoint_index_,
    goal_pose_.position.x, 
    goal_pose_.position.y,
    distance,
    angle_diff * 180.0 / M_PI);
}

bool GotoWaypoint::isCloseToWaypoint(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target, double tolerance)
{
  double distance = calculateDistance(current, target);
  return distance < tolerance;
}

void GotoWaypoint::moveToNextWaypoint()
{
  current_waypoint_index_++;
  
  if (current_waypoint_index_ >= current_path_.size()) {
    // Reached the end of the path
    stopMotion();
    motion_state_ = MotionState::COMPLETED;
    RCLCPP_INFO(this->get_logger(), "Path following completed! Reached all %zu waypoints", current_path_.size());
    return;
  }
  
  // Set next waypoint as goal
  goal_pose_ = current_path_[current_waypoint_index_].pose;
  goal_frame_ = current_path_[current_waypoint_index_].header.frame_id;
  
  const auto& current_pose = current_odom_.pose.pose;
  double distance = calculateDistance(current_pose, goal_pose_);
  double target_yaw = calculateTargetYaw(current_pose, goal_pose_);
  double current_yaw = calculateYawFromQuaternion(current_pose.orientation);
  double angle_diff = calculateAngleDifference(current_yaw, target_yaw);
  
  RCLCPP_INFO(this->get_logger(), 
    "Moving to next waypoint %zu/%zu: (%.2f, %.2f) - distance: %.2fm, angle: %.1f°",
    current_waypoint_index_ + 1, current_path_.size(),
    goal_pose_.position.x, 
    goal_pose_.position.y,
    distance,
    angle_diff * 180.0 / M_PI);
}

}  // namespace astar_pathplanner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<astar_pathplanner::GotoWaypoint>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}