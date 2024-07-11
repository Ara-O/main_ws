import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace, SetParameter
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the robot launch file
    one_robot_launch_file = os.path.join(get_package_share_directory('udm_pioneer_p3dx_main'), 'p3dx_gazebo', 'one_robot.launch.py')

    # Function to create a robot group
    def create_robot_group(robot_name, tf_prefix, init_pose):
        return GroupAction(
            actions=[
                PushRosNamespace(robot_name),
                # SetParameter('frame_prefix', tf_prefix),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([one_robot_launch_file]),
                    launch_arguments={
                        'init_pose': init_pose,
                        'robot_name': robot_name
                    }.items()
                )
            ]
        )

    # Create the robot groups
    robot1_group = create_robot_group('robot1', 'robot1_tf', '-x -1.945 -y -0.50751 -z 0.0')
    robot2_group = create_robot_group('robot2', 'robot2_tf', '-x 1.7558 -y 1.6492 -z 0.0')

    # Create the launch description and populate it
    ld = LaunchDescription()
    ld.add_action(robot1_group)
    ld.add_action(robot2_group)

    return ld
