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
    
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'map', '10x10', 'map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='0.2',
        description='Forward speed for the robot (m/s)'
    )
    
    rotational_speed_arg = DeclareLaunchArgument(
        'rotational_speed', 
        default_value='0.5',
        description='Rotational speed for the robot (rad/s)'
    )
    
    map_server_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('astar_pathplanner'),
            'launch',
            'map_server.launch.py'
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    astar_pathplanner_node = Node(
        package='astar_pathplanner',
        executable='astar_pathplanner_node',
        name='astar_pathplanner',
        output='screen',
        parameters=[
            {
                'heuristic_weight': 1.0,
                'use_diagonal_movement': True,
                'allow_unknown': True,
                'obstacle_cost_threshold': 65.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    
    goto_waypoint_node = Node(
        package='astar_pathplanner',
        executable='goto_waypoint',
        name='goto_waypoint',
        output='screen',
        parameters=[
            {
                'forward_speed': LaunchConfiguration('forward_speed'),
                'rotational_speed': LaunchConfiguration('rotational_speed'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
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
    
    rviz_node_delayed = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )
    
    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        forward_speed_arg,
        rotational_speed_arg,
        astar_pathplanner_node,
        goto_waypoint_node,
        # map_server_launch,
        rviz_node
        # rviz_node_delayed,
    ])
