from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    p3dx_navigation_dir = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('no_static_map', default_value='true'),
        DeclareLaunchArgument('odom_frame_id', default_value='robot2_tf/odom'),
        DeclareLaunchArgument('base_frame_id', default_value='robot2_tf/base_link'),
        DeclareLaunchArgument('global_frame_id', default_value='robot2_tf/odom'),
        DeclareLaunchArgument('odom_topic', default_value='/robot2/odom'),
        DeclareLaunchArgument('laser_topic', default_value='/robot2/scan'),

        # Include the nav2_bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(p3dx_navigation_dir, 'config', 'nav2_params.yaml'),
                'map': os.path.join(p3dx_navigation_dir, 'maps', 'feb2_map.yaml')
            }.items()
        ),
    ])