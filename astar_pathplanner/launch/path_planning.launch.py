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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    
    pkg_share = get_package_share_directory('astar_pathplanner')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    astar_pathplanner_node = Node(
        package='astar_pathplanner',
        executable='astar_pathplanner_node',
        name='astar_pathplanner',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'astar_pathplanner.yaml')],
    )
    
    goto_waypoint_node = Node(
        package='astar_pathplanner',
        executable='goto_waypoint',
        name='goto_waypoint',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'goto_waypoint.yaml')],
        remappings=[
            ('/goal_pose', '/move_base_simple/goal'),
            # ('/plan', '/global_planner/plan'),
        ]
    )
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('astar_pathplanner'),
        'rviz',
        'astar_pathplanner.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        astar_pathplanner_node,
        goto_waypoint_node,
        rviz_node
    ])
