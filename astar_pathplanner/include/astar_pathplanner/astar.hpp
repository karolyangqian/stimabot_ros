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


#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace astar_pathplanner
{

struct AStarNode
{
  int x, y;          // Grid coordinates
  double f, g, h;    // A* costs
  AStarNode* parent; // Parent node for path reconstruction
  
  AStarNode(int x = 0, int y = 0, double g = 0.0, double h = 0.0, AStarNode* parent = nullptr)
    : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}
  
  bool operator>(const AStarNode& other) const {
    return f > other.f;
  }
};

struct NodeHash {
  std::size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<int>{}(p.first) ^ (std::hash<int>{}(p.second) << 1);
  }
};

class AStar
{
public:
  AStar();
  ~AStar() = default;

  // Configuration methods
  void setHeuristicWeight(double weight);
  void setUseDiagonalMovement(bool use_diagonal);
  void setAllowUnknown(bool allow_unknown);
  void setObstacleCostThreshold(double threshold);
  void setMap(const nav_msgs::msg::OccupancyGrid& map);

  // Core A* algorithm
  std::vector<std::pair<int, int>> findPath(
    int start_x, int start_y, int goal_x, int goal_y);

  // Coordinate conversion functions
  bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;
  void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const;

  // Path processing
  nav_msgs::msg::Path gridPathToWorldPath(
    const std::vector<std::pair<int, int>>& grid_path,
    const std::string& frame_id) const;
  std::vector<std::pair<int, int>> smoothPath(
    const std::vector<std::pair<int, int>>& path) const;

  // Utility functions
  bool isValidCell(int x, int y) const;
  double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) const;

private:
  // Algorithm parameters
  double heuristic_weight_;
  bool use_diagonal_movement_;
  bool allow_unknown_;
  double obstacle_cost_threshold_;
  
  // Grid parameters
  nav_msgs::msg::OccupancyGrid current_map_;
  double grid_resolution_;  // meters per cell
  int grid_width_;
  int grid_height_;
  double origin_x_;
  double origin_y_;
  bool has_map_;

  // Core A* algorithm helper methods
  double calculateHeuristic(int x1, int y1, int x2, int y2) const;
  std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;
  double getMovementCost(int x1, int y1, int x2, int y2) const;
  std::vector<std::pair<int, int>> reconstructPath(AStarNode* goal_node) const;
  bool hasLineOfSight(
    const std::pair<int, int>& start, 
    const std::pair<int, int>& end) const;
};

}  // namespace astar_pathplanner

#endif  // ASTAR_PATHPLANNER__ASTAR_HPP_
