import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    amcl_config_robot1 = os.path.join(get_package_share_directory('udm_pioneer_p3dx', 'p3dx_navigation', "amcl_config.yaml"))
    amcl_config_robot2 = os.path.join(get_package_share_directory('udm_pioneer_p3dx', 'p3dx_navigation', "amcl_config.yaml"))
    map_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx', 'p3dx_navigation', 'map_lab.yaml'))

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }, {'yaml_filename': map_file}]
        ),
        
        Node(
            namespace="robot1",
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_robot1]
        ),

        Node(
            namespace="robot2",
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_robot2]
        ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager'
        )
    ])