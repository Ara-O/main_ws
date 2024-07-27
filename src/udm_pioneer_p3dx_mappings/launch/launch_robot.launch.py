import os
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description="True if using simulation"
    )

    default_slam_config_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx_mappings'), 'config', 'mapper_params_online_async.yaml' )

    slam_arg = DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_config_file,
            description="The config file that will be used when launching the slam toolbox"
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Allow the user to pass in any world file they would like to spawn the robot in
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(get_package_share_directory('udm_pioneer_p3dx_mappings'), 'maps', 'default_map.world'),
        description="The world file to map"
    )
    
    world_file = LaunchConfiguration('world_file')
    
    #Gets the robot xacro file
    robot_xacro_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx_mappings'), 'pioneer_p3dx_description', 'urdf', 'pioneer3dx.xacro')
    
    #Process the xacro file into a urdf file
    robot_description_config = xacro.process_file(robot_xacro_file)

    # Gets the robot's urdf and pass it as a parameter to the robot description
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    # Setting up robot state publisher, passing the robot's urdf as the parameter
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Setting up robot joint publisher
    node_robot_joint_publisher =  Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': False, 'use_sim_time': True}]
    )

    #Include the gazebo launch file 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'pioneer_p3dx_robot', '-topic', 'robot_description', 
                   ],
        output='screen'
    )

    slam_params_file = LaunchConfiguration('slam_params_file')
    # Set up slam
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[slam_params_file, use_sim_time]
    )

    # Get rviz config
    rviz_config_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx_mappings'), 'config', 'robot.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    )

    # Return the launch description
    return LaunchDescription([
        sim_time_arg,
        slam_arg,
        world_file_arg,
        node_robot_state_publisher,
        node_robot_joint_publisher,
        gazebo,
        slam_node,
        rviz,
        spawn_entity,
    ])