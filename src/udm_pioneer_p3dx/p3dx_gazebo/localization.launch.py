import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    amcl_config_robot1 = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', "amcl_config_robot1.yaml")
    amcl_config_robot2 = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', "amcl_config_robot2.yaml")
    default_map_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'maps', 'feb2_world.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map_file,
            description="The map that the robot will be localized in"
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name':  'map'}, 
                        {'frame_id': 'map'},
                        {'yaml_filename': LaunchConfiguration('map_file')}]
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
            parameters=[amcl_config_robot2],
                        # arguments=['--ros-args', '--log-level', 'debug']

        ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True}, {'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': ['map_server', 'robot1/amcl', 'robot2/amcl']}]
        )
    ])