#  Copyright 2025 Karol Yangqian Poetracahya

#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


cmake_minimum_required(VERSION 3.8)
project(astar_pathplanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ===========================================================================
# A* Algorithm library
# ===========================================================================
add_library(astar_core
  src/astar.cpp
)

target_include_directories(astar_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(astar_core ${dependencies})

# ===========================================================================
# A* path planner node
# ===========================================================================
add_executable(astar_pathplanner_node
  src/astar_pathplanner.cpp
)

target_link_libraries(astar_pathplanner_node astar_core)

ament_target_dependencies(astar_pathplanner_node ${dependencies})

target_include_directories(astar_pathplanner_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ===========================================================================
# Waypoint navigation executable
# ===========================================================================
add_executable(goto_waypoint
  src/goto_waypoint.cpp
)

ament_target_dependencies(goto_waypoint ${dependencies})

target_include_directories(goto_waypoint
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ===========================================================================
# Install targets and directories
# ===========================================================================

# Install library properly
install(TARGETS astar_core
  EXPORT astar_coreTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS astar_pathplanner_node goto_waypoint
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.hpp"
)

# Install other directories
install(DIRECTORY
  map
  models
  launch
  worlds
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export library targets and dependencies
ament_export_targets(astar_coreTargets HAS_LIBRARY_TARGET)
ament_export_libraries(astar_core)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()