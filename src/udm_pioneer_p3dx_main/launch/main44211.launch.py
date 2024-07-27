import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # Path to our custom world file
    world_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx_main'), 'worlds', 'feb2_scenario.world')

    # Declare the launch arguments, with the defaults
    declared_arguments = [
        DeclareLaunchArgument('paused', default_value='false', description='Start the simulation paused'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('gui', default_value='true', description='Run with GUI'),
        DeclareLaunchArgument('headless', default_value='false', description='Run headless (no GUI)'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),
        DeclareLaunchArgument('world_file', default_value=world_file, description='Enter the path of the world file you would like to launch these robots in')
    ]

    # Path to the gazebo_ros launch file, so it connects ros with gazebo
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    gazebo_launch_file = os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')

    # Include the Gazebo launch file - the empty_world.launch equivalent in ROS
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            'world': world_file,
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'headless': LaunchConfiguration('headless'),
            'debug': LaunchConfiguration('debug'),
	        'verbose': 'true'
        }.items()
    )

    # Starts the script to spawn the robots
    p3dx_robot_launch_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx_main'), 'p3dx_gazebo', 'robots44211.launch.py')

    robot_launch = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([p3dx_robot_launch_file])
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(get_package_share_directory('udm_pioneer_p3dx_mappings'), 'pioneer_p3dx_description', 'config', 'robot.rviz')]
    )

    # Return the LaunchDescription
    return LaunchDescription(declared_arguments + [start_gazebo, robot_launch, start_rviz])
