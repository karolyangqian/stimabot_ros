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
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def check_map_file(context, *args, **kwargs):
    """Check if the map file exists and log the result"""
    map_path = context.launch_configurations['map']
    
    if os.path.exists(map_path):
        # Also check if the corresponding .pgm file exists
        yaml_dir = os.path.dirname(map_path)
        pgm_files = [f for f in os.listdir(yaml_dir) if f.endswith('.pgm')]
        
        return [
            LogInfo(msg=f"✅ Map YAML found: {map_path}"),
            LogInfo(msg=f"📁 Map directory: {yaml_dir}"),
            LogInfo(msg=f"🖼️  PGM files found: {pgm_files}"),
            LogInfo(msg="🚀 Map server should start successfully!")
        ]
    else:
        return [
            LogInfo(msg=f"❌ Map file NOT found: {map_path}"),
            LogInfo(msg=f"📁 Expected directory: {os.path.dirname(map_path)}"),
            LogInfo(msg="⚠️  Map server will fail to start!")
        ]

def generate_launch_description():
    
    pkg_share = get_package_share_directory('astar_pathplanner')
    default_map_path = os.path.join(pkg_share, 'map', '10x10', 'map.yaml')

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    package_info = LogInfo(
        msg=f"🗺️  Starting Map Server from: {pkg_share}"
    )
    
    debug_info = LogInfo(
        msg="🔧 Debug: Use 'ros2 topic echo /map --once' to verify map publication"
    )
    
    map_file_check = OpaqueFunction(function=check_map_file)
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'topic_name': 'map',
                'frame_id': 'map'
            }
        ]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'node_names': ['map_server'],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'bond_timeout': 4.0
            }
        ]
    )
    
    status_check = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="📊 Checking map server status after 5 seconds..."),
            LogInfo(msg="🔍 Run: ros2 topic list | grep map"),
            LogInfo(msg="🔍 Run: ros2 topic echo /map --once")
        ]
    )
    
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        package_info,
        map_file_check,
        debug_info,
        map_server_node,
        lifecycle_manager_node,
        static_transform_publisher,
        status_check
    ])