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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Goto Waypoint Node
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
    
    return LaunchDescription([
        forward_speed_arg,
        rotational_speed_arg,
        use_sim_time_arg,
        goto_waypoint_node
    ])
