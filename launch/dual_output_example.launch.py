#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cell_size',
            default_value='0.5',
            description='Cell size for the grid map [m/cell]'
        ),
        DeclareLaunchArgument(
            'length_x',
            default_value='20.0',
            description='Length in x-direction [m]'
        ),
        DeclareLaunchArgument(
            'length_y',
            default_value='30.0',
            description='Length in y-direction [m]'
        ),
        DeclareLaunchArgument(
            'cloud_in_topic',
            default_value='nonground',
            description='Input point cloud topic'
        ),
        
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            name='pointcloud_to_grid_dual',
            parameters=[{
                'cell_size': LaunchConfiguration('cell_size'),
                'length_x': LaunchConfiguration('length_x'),
                'length_y': LaunchConfiguration('length_y'),
                'cloud_in_topic': LaunchConfiguration('cloud_in_topic'),
                # OccupancyGrid topics
                'mapi_topic_name': 'intensity_grid',
                'maph_topic_name': 'height_grid',
                # GridMap topics
                'mapi_gridmap_topic_name': 'intensity_gridmap',
                'maph_gridmap_topic_name': 'height_gridmap',
                'verbose1': True,
            }],
            output='screen'
        )
    ])
