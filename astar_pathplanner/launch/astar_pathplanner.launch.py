#!/usr/bin/env python3

#    Copyright 2025 Karol Yangqian Poetracahya

#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments
    heuristic_weight_arg = DeclareLaunchArgument(
        'heuristic_weight',
        default_value='1.0',
        description='Weight for heuristic function (higher = more greedy)'
    )
    
    use_diagonal_movement_arg = DeclareLaunchArgument(
        'use_diagonal_movement', 
        default_value='true',
        description='Allow diagonal movement in pathfinding'
    )
    
    allow_unknown_arg = DeclareLaunchArgument(
        'allow_unknown',
        default_value='true',
        description='Allow traversing unknown cells'
    )
    
    obstacle_cost_threshold_arg = DeclareLaunchArgument(
        'obstacle_cost_threshold',
        default_value='65.0',
        description='Occupancy threshold for obstacles (0-100)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # A* Pathplanner Node
    astar_pathplanner_node = Node(
        package='astar_pathplanner',
        executable='astar_pathplanner_node',
        name='astar_pathplanner',
        output='screen',
        parameters=[
            {
                'heuristic_weight': LaunchConfiguration('heuristic_weight'),
                'use_diagonal_movement': LaunchConfiguration('use_diagonal_movement'),
                'allow_unknown': LaunchConfiguration('allow_unknown'),
                'obstacle_cost_threshold': LaunchConfiguration('obstacle_cost_threshold'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            # Can remap topics if needed
            # ('/goal_pose', '/move_base_simple/goal'),
        ]
    )
    
    return LaunchDescription([
        heuristic_weight_arg,
        use_diagonal_movement_arg,
        allow_unknown_arg,
        obstacle_cost_threshold_arg,
        use_sim_time_arg,
        astar_pathplanner_node
    ])
