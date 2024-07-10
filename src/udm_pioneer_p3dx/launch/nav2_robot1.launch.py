import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file]),
            launch_arguments={
                'map': os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'maps', 'map_lab.yaml'),
                'namespace': 'robot1',
                'use_sim_time': 'true',
                'params_file': os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'config', 'nav2_params.yaml')
            }.items()
        )
    ])
