import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gazebo_ros_pkg = get_package_share_directory('udm_pioneer_p3dx')

    movebase_robot1 = os.path.join(gazebo_ros_pkg, 'p3dx_gazebo', 'move_base_robot1.launch.py')
    # movebase_robot2 = os.path.join(gazebo_ros_pkg, 'p3dx_gazebo','move_base_robot2.launch.py')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([movebase_robot1]),
            launch_arguments={'use_sim_time': 'true'}.items()
        )

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([movebase_robot2]),
        #     launch_arguments={'use_sim_time': 'true'}.items()
        # ),
    ])
