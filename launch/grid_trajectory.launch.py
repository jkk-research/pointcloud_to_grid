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
                {'position_x': -20.0},
                {'position_y': 0.0},
                {'verbose1': False},
                {'verbose2': False},
                {'cell_size': 0.5},
                {'length_x': 40.0},
                {'length_y': 60.0},
                #{'frame_out': 'os1_sensor'},
                {'mapi_topic_name': 'intensity_grid'},
                {'maph_topic_name': 'height_grid'},
            ]
        )

    ])

