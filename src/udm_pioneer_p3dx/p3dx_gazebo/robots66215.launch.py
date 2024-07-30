import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the robot launch file
    one_robot_launch_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_gazebo', 'one_robot.launch.py')
    default_map_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'maps', 'feb2_world.yaml')


    # Function to create a robot group
    def create_robot_group(robot_index, robot_name, tf_prefix, init_pose):
        # nav_launch_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'launch', f'nav2_{robot_name}.launch.py')
        return GroupAction(
            actions=[
                # PushRosNamespace(robot_name),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([one_robot_launch_file]),
                    launch_arguments={
                        'init_pose': init_pose,
                        'robot_name': robot_name,
                        'robot_index': robot_index,
                        'model': os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'urdf', robot_name+'.xacro') 
                    }.items()
                ),
            ]
        )
  
    # Create the robot groups
    robot1_group = create_robot_group('1', 'robot1', 'robot1_tf', '-x -2.2366 -y 0.97246 -z 0.0')
    robot2_group = create_robot_group('2', 'robot2', 'robot2_tf', '-x 1.2054 -y -1.7809 -z 0.0')

    # Create the launch description and populate it
    ld = LaunchDescription([
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
                        {'topic_name': 'map'},
                        {'frame_id': 'map'},
                        {'yaml_filename': LaunchConfiguration('map_file')}]
        ),
        robot1_group,
        robot2_group,
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'bond_timeout': 0.0},
                        {'autostart': True},
                        {'node_names': ['map_server', 'robot1/amcl', 'robot2/amcl']}]
        )
    ])

    return ld
