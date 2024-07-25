import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def spawn_robot(context, *args, **kwargs):
    # Get the initialization pose
    init_pose = LaunchConfiguration('init_pose').perform(context).split()
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_index = LaunchConfiguration('robot_index').perform(context)

    amcl_config_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', "amcl_config_robot" + robot_index + ".yaml")

    # Extract x, y, z values
    x = init_pose[1]
    y = init_pose[3]
    z = init_pose[5]

    # Define nodes
    urdf_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        namespace=robot_name,
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-file', LaunchConfiguration('model'),
            '-x', x,
            '-y', y,
            '-z', z,
            '-robot_namespace', LaunchConfiguration('robot_name')
        ],
        parameters=[{'use_sim_time': True}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{'use_gui': False, 'use_sim_time': True, 'frame_prefix': robot_name + "/"}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_name,
        parameters=[{'use_sim_time': True, 
                     'frame_prefix': robot_name + "/",  
                     'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    amcl_node = Node(
        namespace=robot_name,
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config_file,
                    {'initial_pose': {'x':  float(x)}},
                    {'initial_pose': {'y':  float(y)}}]
    )

    return [urdf_spawner, joint_state_publisher, robot_state_publisher, amcl_node]

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('robot_name', description='Name of the robot'),
        DeclareLaunchArgument('robot_index', description='Index of the robot'),
        DeclareLaunchArgument('init_pose', description='Initial pose of the robot'),
        DeclareLaunchArgument('model', description='URDF model file')
    ]

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Add the function to spawn the robot
    ld.add_action(OpaqueFunction(function=spawn_robot))

    return ld
