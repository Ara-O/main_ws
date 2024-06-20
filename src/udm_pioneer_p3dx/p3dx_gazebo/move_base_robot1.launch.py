import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
    p3dx_navigation_dir = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation')
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch',
            name='nav2_bringup',
            output='screen',
            parameters=[os.path.join(p3dx_navigation_dir, 'config', 'nav2_params.yaml')]
        )
    ])
