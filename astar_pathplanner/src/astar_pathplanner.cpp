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


#include "astar_pathplanner/astar_pathplanner.hpp"

namespace astar_pathplanner
{

AStarPathplanner::AStarPathplanner() : Node("astar_pathplanner")
{
  // Declare parameters
  this->declare_parameter("heuristic_weight", 1.0);
  this->declare_parameter("use_diagonal_movement", true);
  this->declare_parameter("allow_unknown", true);
  this->declare_parameter("obstacle_cost_threshold", 65.0);
  
  // Get parameters and configure A* algorithm
  double heuristic_weight;
  bool use_diagonal_movement;
  bool allow_unknown;
  double obstacle_cost_threshold;
  
  this->get_parameter("heuristic_weight", heuristic_weight);
  this->get_parameter("use_diagonal_movement", use_diagonal_movement);
  this->get_parameter("allow_unknown", allow_unknown);
  this->get_parameter("obstacle_cost_threshold", obstacle_cost_threshold);
  
  // Configure A* algorithm
  astar_.setHeuristicWeight(heuristic_weight);
  astar_.setUseDiagonalMovement(use_diagonal_movement);
  astar_.setAllowUnknown(allow_unknown);
  astar_.setObstacleCostThreshold(obstacle_cost_threshold);
  
  RCLCPP_INFO(this->get_logger(), "A* Pathplanner Node initialized");
  RCLCPP_INFO(this->get_logger(), "Heuristic weight: %.2f", heuristic_weight);
  RCLCPP_INFO(this->get_logger(), "Use diagonal movement: %s", use_diagonal_movement ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Allow unknown: %s", allow_unknown ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Obstacle cost threshold: %.0f", obstacle_cost_threshold);
  
  // Initialize state
  has_odom_ = false;
  has_map_ = false;
  
  // Create subscribers
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&AStarPathplanner::goalPoseCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&AStarPathplanner::odomCallback, this, std::placeholders::_1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&AStarPathplanner::mapCallback, this, std::placeholders::_1));
  
  // Create publisher
  plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
  
  RCLCPP_INFO(this->get_logger(), "Waiting for odometry, map, and goal pose...");
}

void AStarPathplanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
  has_odom_ = true;
}

void AStarPathplanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Pass the map to the A* algorithm
  astar_.setMap(*msg);
  has_map_ = true;
  
  RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f", 
              msg->info.width, msg->info.height, msg->info.resolution);
}

void AStarPathplanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!has_odom_ || !has_map_) {
    RCLCPP_WARN(this->get_logger(), "Goal received but missing odometry or map data");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Goal pose received: (%.2f, %.2f)", 
              msg->pose.position.x, msg->pose.position.y);
  
  // Get current position from odometry
  double start_x = current_odom_.pose.pose.position.x;
  double start_y = current_odom_.pose.pose.position.y;
  double goal_x = msg->pose.position.x;
  double goal_y = msg->pose.position.y;
  
  RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
              start_x, start_y, goal_x, goal_y);
  
  // Convert to grid coordinates
  int start_grid_x, start_grid_y, goal_grid_x, goal_grid_y;
  
  if (!astar_.worldToGrid(start_x, start_y, start_grid_x, start_grid_y)) {
    RCLCPP_ERROR(this->get_logger(), "Start position (%.2f, %.2f) is outside map bounds", start_x, start_y);
    return;
  }
  
  if (!astar_.worldToGrid(goal_x, goal_y, goal_grid_x, goal_grid_y)) {
    RCLCPP_ERROR(this->get_logger(), "Goal position (%.2f, %.2f) is outside map bounds", goal_x, goal_y);
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Grid coordinates: start(%d, %d), goal(%d, %d)",
              start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);
  
  // Find path using A* algorithm
  std::vector<std::pair<int, int>> grid_path = astar_.findPath(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);
  
  if (grid_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No path found!");
    return;
  }
  
  // Convert grid path to world path
  nav_msgs::msg::Path world_path = astar_.gridPathToWorldPath(grid_path, msg->header.frame_id);
  world_path.header.stamp = this->now();
  
  // Publish the path
  plan_pub_->publish(world_path);
  
  RCLCPP_INFO(this->get_logger(), "Path published with %zu waypoints", world_path.poses.size());
}

}  // namespace astar_pathplanner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<astar_pathplanner::AStarPathplanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
