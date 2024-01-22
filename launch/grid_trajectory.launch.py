from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
        Node(
            package='pointcloud_to_grid',
            executable='grid_trajectory',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("topic")},
                {'position_x': -20.0},  # meters
                {'position_y': 0.0},  # meters
                {'verbose1': False},
                {'verbose2': False},
                {'cell_size': 0.2},  # meters
                {'length_x': 40.0},  # meters
                {'length_y': 60.0},  # meters
                #{'frame_out': 'os1_sensor'},
                {'mapi_topic_name': 'intensity_grid'},
                {'maph_topic_name': 'height_grid'},
                {'search_length': 4.0}, # meters
                {'search_range_deg': 80.0}, # degrees
                {'search_resolution_deg': 0.5}, # degrees
                {'search_start_mid_deg': -180.0}, # degrees
            ]
        )

    ])

