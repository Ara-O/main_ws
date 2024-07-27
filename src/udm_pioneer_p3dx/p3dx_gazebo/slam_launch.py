import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_config_robot1 = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', "slam_config_robot1.yaml")
    slam_config_robot2 = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', "slam_config_robot2.yaml")
    # map_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'maps', 'group3_map_project_1.yaml')

    return LaunchDescription([        
        Node(
            namespace="robot1",
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam',
            output='screen',
            parameters=[slam_config_robot1, {'use_sim_time': True}]
        ),

        Node(
            namespace="robot2",
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam',
            output='screen',
            parameters=[slam_config_robot2, {'use_sim_time': True}]
        ),


        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}, {'autostart': True},
        #                 {'bond_timeout': 0.0},
        #                 {'node_names': ['robot1/slam', 'robot2/slam']}]
        # )
    ])
