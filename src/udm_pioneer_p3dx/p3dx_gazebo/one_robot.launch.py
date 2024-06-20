import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def spawn_robot(context, *args, **kwargs):
    # Get the initialization pose
    init_pose = LaunchConfiguration('init_pose').perform(context).split()
    
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
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-file', LaunchConfiguration('model'),
            '-x', x,
            '-y', y,
            '-z', z,
            '-robot_namespace', LaunchConfiguration('robot_name')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': False}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    return [urdf_spawner, joint_state_publisher, robot_state_publisher]

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('robot_name', description='Name of the robot'),
        DeclareLaunchArgument('init_pose', description='Initial pose of the robot'),
        DeclareLaunchArgument('model', default_value=os.path.join(
            get_package_share_directory('udm_pioneer_p3dx'), 'urdf', 'pioneer3dx.xacro'), description='URDF model file')
    ]

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Add the function to spawn the robot
    ld.add_action(OpaqueFunction(function=spawn_robot))

    return ld

