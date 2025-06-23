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


#include "astar_pathplanner/astar.hpp"
#include <algorithm>

namespace astar_pathplanner
{
AStar::AStar()
  : heuristic_weight_(1.0)
  , use_diagonal_movement_(true)
  , allow_unknown_(true)
  , obstacle_cost_threshold_(65.0)
  , grid_resolution_(0.05)
  , grid_width_(0)
  , grid_height_(0)
  , origin_x_(0.0)
  , origin_y_(0.0)
  , has_map_(false)
{
}

void AStar::setHeuristicWeight(double weight)
{
  heuristic_weight_ = weight;
}

void AStar::setUseDiagonalMovement(bool use_diagonal)
{
  use_diagonal_movement_ = use_diagonal;
}

void AStar::setAllowUnknown(bool allow_unknown)
{
  allow_unknown_ = allow_unknown;
}

void AStar::setObstacleCostThreshold(double threshold)
{
  obstacle_cost_threshold_ = threshold;
}

void AStar::setMap(const nav_msgs::msg::OccupancyGrid& map)
{
  current_map_ = map;
  grid_resolution_ = map.info.resolution;
  grid_width_ = map.info.width;
  grid_height_ = map.info.height;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;
  has_map_ = true;
}

std::vector<std::pair<int, int>> AStar::findPath(
  int start_x, int start_y, int goal_x, int goal_y)
{
  std::vector<std::pair<int, int>> path;
  
  if (!has_map_) {
    return path; // Empty path if no map
  }
  
  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
  
  std::unordered_map<std::pair<int, int>, bool, NodeHash> closed_set;
  
  std::unordered_map<std::pair<int, int>, double, NodeHash> g_costs;
  
  std::unordered_map<std::pair<int, int>, AStarNode*, NodeHash> all_nodes;
  
  double start_h = calculateHeuristic(start_x, start_y, goal_x, goal_y) * heuristic_weight_;
  AStarNode* start_node = new AStarNode(start_x, start_y, 0.0, start_h, nullptr);
  
  open_set.push(*start_node);
  g_costs[{start_x, start_y}] = 0.0;
  all_nodes[{start_x, start_y}] = start_node;
  
  int nodes_explored = 0;
  
  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();
    nodes_explored++;
    
    std::pair<int, int> current_pos = {current.x, current.y};
    
    if (closed_set.find(current_pos) != closed_set.end()) {
      continue;
    }
    
    closed_set[current_pos] = true;
    
    if (current.x == goal_x && current.y == goal_y) {
      std::vector<std::pair<int, int>> raw_path = reconstructPath(all_nodes[current_pos]);
      
      path = smoothPath(raw_path);
      
      for (auto& node_pair : all_nodes) {
        delete node_pair.second;
      }
      
      return path;
    }
    
    // Explore neighbors
    std::vector<std::pair<int, int>> neighbors = getNeighbors(current.x, current.y);
    
    for (const auto& neighbor_pos : neighbors) {
      int nx = neighbor_pos.first;
      int ny = neighbor_pos.second;
      
      // Skip if in closed set or invalid
      if (closed_set.find({nx, ny}) != closed_set.end()) {
        continue;
      }
      
      // Calculate tentative g cost
      double movement_cost = getMovementCost(current.x, current.y, nx, ny);
      double tentative_g = current.g + movement_cost;
      
      // Check if this path to neighbor is better
      auto g_cost_it = g_costs.find({nx, ny});
      bool is_better_path = false;
      
      if (g_cost_it == g_costs.end()) {
        // First time visiting this node
        is_better_path = true;
      } else if (tentative_g < g_cost_it->second) {
        // Found a better path to this node
        is_better_path = true;
      }
      
      if (is_better_path) {
        // Update g cost
        g_costs[{nx, ny}] = tentative_g;
        
        // Calculate heuristic
        double h = calculateHeuristic(nx, ny, goal_x, goal_y) * heuristic_weight_;
        
        // Create or update node
        AStarNode* neighbor_node;
        auto node_it = all_nodes.find({nx, ny});
        if (node_it == all_nodes.end()) {
          neighbor_node = new AStarNode(nx, ny, tentative_g, h, all_nodes[current_pos]);
          all_nodes[{nx, ny}] = neighbor_node;
        } else {
          neighbor_node = node_it->second;
          neighbor_node->g = tentative_g;
          neighbor_node->h = h;
          neighbor_node->f = tentative_g + h;
          neighbor_node->parent = all_nodes[current_pos];
        }
        
        // Add to open set
        open_set.push(*neighbor_node);
      }
    }
  }
  
  // No path found - clean up allocated nodes
  for (auto& node_pair : all_nodes) {
    delete node_pair.second;
  }
  
  return path; // Return empty path
}

bool AStar::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const
{
  if (!has_map_) {
    return false;
  }
  
  // Convert world coordinates to grid coordinates
  grid_x = static_cast<int>((world_x - origin_x_) / grid_resolution_);
  grid_y = static_cast<int>((world_y - origin_y_) / grid_resolution_);
  
  // Check if coordinates are within bounds
  return (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_);
}

void AStar::gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const
{
  // Convert grid coordinates to world coordinates (center of cell)
  world_x = origin_x_ + (grid_x + 0.5) * grid_resolution_;
  world_y = origin_y_ + (grid_y + 0.5) * grid_resolution_;
}

nav_msgs::msg::Path AStar::gridPathToWorldPath(
  const std::vector<std::pair<int, int>>& grid_path,
  const std::string& frame_id) const
{
  nav_msgs::msg::Path world_path;
  world_path.header.frame_id = frame_id;
  
  for (size_t i = 0; i < grid_path.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = world_path.header;
    
    gridToWorld(grid_path[i].first, grid_path[i].second, 
               pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    
    // Calculate orientation based on path direction
    if (i < grid_path.size() - 1) {
      // Calculate direction to next waypoint
      double next_x, next_y;
      gridToWorld(grid_path[i + 1].first, grid_path[i + 1].second, next_x, next_y);
      
      double dx = next_x - pose.pose.position.x;
      double dy = next_y - pose.pose.position.y;
      double yaw = atan2(dy, dx);
      
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
    } else {
      // For the last point, use the previous orientation or identity
      if (!world_path.poses.empty()) {
        pose.pose.orientation = world_path.poses.back().pose.orientation;
      } else {
        pose.pose.orientation.w = 1.0;
      }
    }
    
    world_path.poses.push_back(pose);
  }
  
  return world_path;
}

std::vector<std::pair<int, int>> AStar::smoothPath(
  const std::vector<std::pair<int, int>>& path) const
{
  if (path.size() <= 2) {
    return path; // Can't smooth paths with 2 or fewer points
  }
  
  std::vector<std::pair<int, int>> smoothed_path;
  smoothed_path.push_back(path[0]); // Always keep start point
  
  // Simple line-of-sight smoothing
  size_t current_idx = 0;

  while (current_idx < path.size() - 1) {
    size_t furthest_visible = current_idx + 1;

    // Find the furthest point we can see from current position
    for (size_t i = current_idx + 2; i < path.size(); ++i) {
      if (hasLineOfSight(path[current_idx], path[i])) {
        furthest_visible = i;
      } else {
        break;
      }
    }
    
    // Add the furthest visible point
    if (furthest_visible != current_idx) {
      smoothed_path.push_back(path[furthest_visible]);
      current_idx = furthest_visible;
    } else {
      // If we can't see further, just move to next point
      current_idx++;
      if (current_idx < path.size()) {
        smoothed_path.push_back(path[current_idx]);
      }
    }
  }
  
  return smoothed_path;
}

bool AStar::isValidCell(int x, int y) const
{
  if (!has_map_) {
    return false;
  }
  
  // Check bounds
  if (x < 0 || x >= grid_width_ || y < 0 || y >= grid_height_) {
    return false;
  }
  
  // Get occupancy value from map
  int index = y * grid_width_ + x;
  if (index >= static_cast<int>(current_map_.data.size())) {
    return false;
  }
  
  int8_t occupancy = current_map_.data[index];
  
  // Occupancy grid values:
  // -1: unknown
  //  0: free
  //  1-100: occupied (probability)
    
  if (occupancy >= static_cast<int8_t>(obstacle_cost_threshold_)) {
    return false; // Occupied
  }

  if (occupancy == -1 && !allow_unknown_) {
    return false; // Unknown and not allowed
  }
  
  return true;
}

double AStar::calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) const
{
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 mat(tf_quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

// Private helper methods

double AStar::calculateHeuristic(int x1, int y1, int x2, int y2) const
{
  double dx = abs(x1 - x2);
  double dy = abs(y1 - y2);
  
  if (use_diagonal_movement_) {
    // Diagonal distance (Chebyshev distance with diagonal cost)
    return std::max(dx, dy) + (sqrt(2.0) - 1.0) * std::min(dx, dy);
  } else {
    // Manhattan distance for 4-way movement
    return dx + dy;
  }
}

std::vector<std::pair<int, int>> AStar::getNeighbors(int x, int y) const
{
  std::vector<std::pair<int, int>> neighbors;
  
  std::vector<std::pair<int, int>> directions = {
    {0, 1}, {0, -1}, {1, 0}, {-1, 0}
  };
  
  if (use_diagonal_movement_) {
    directions.push_back({1, 1});
    directions.push_back({1, -1});
    directions.push_back({-1, 1});
    directions.push_back({-1, -1});
  }
  
  for (const auto& dir : directions) {
    int nx = x + dir.first;
    int ny = y + dir.second;
    
    if (isValidCell(nx, ny)) {
      neighbors.push_back({nx, ny});
    }
  }
  
  return neighbors;
}

double AStar::getMovementCost(int x1, int y1, int x2, int y2) const
{
  double dx = abs(x1 - x2);
  double dy = abs(y1 - y2);
  
  double base_cost;
  if (dx == 1 && dy == 1) {
    // Diagonal movement
    base_cost = sqrt(2.0);
  } else {
    // Straight movement
    base_cost = 1.0;
  }
  
  // Add terrain cost based on occupancy values
  int index = y2 * grid_width_ + x2;
  if (index >= static_cast<int>(current_map_.data.size())) {
    return base_cost;
  }
  
  int8_t occupancy = current_map_.data[index];
  
  // Add small penalty for cells with higher occupancy probability
  double terrain_multiplier = 1.0;
  if (occupancy > 0) {
    terrain_multiplier = 1.0 + (occupancy / 100.0) * 0.1; // Up to 10% penalty
  }
  
  return base_cost * terrain_multiplier;
}

std::vector<std::pair<int, int>> AStar::reconstructPath(AStarNode* goal_node) const
{
  std::vector<std::pair<int, int>> path;
  AStarNode* current = goal_node;
  
  while (current != nullptr) {
    path.push_back({current->x, current->y});
    current = current->parent;
  }
  
  // Reverse to get path from start to goal
  std::reverse(path.begin(), path.end());
  return path;
}

bool AStar::hasLineOfSight(
  const std::pair<int, int>& start, 
  const std::pair<int, int>& end) const
{
  // Bresenham's line algorithm for line-of-sight check
  int x0 = start.first, y0 = start.second;
  int x1 = end.first, y1 = end.second;
  
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  
  int x = x0, y = y0;
  
  while (true) {
    // Check if current cell is valid
    if (!isValidCell(x, y)) {
      return false;
    }
    
    if (x == x1 && y == y1) {
      break;
    }
    
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
  
  return true;
}

}  // namespace astar_pathplanner
