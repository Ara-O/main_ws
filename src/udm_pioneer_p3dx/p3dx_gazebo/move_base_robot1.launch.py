from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_directory = LaunchConfiguration('config_directory', default=os.path.join(
        get_package_share_directory('p3dx_navigation'), 'config'))
    
    return LaunchDescription([
        DeclareLaunchArgument('no_static_map', default_value='true'),
        DeclareLaunchArgument('odom_frame_id', default_value='robot1_tf/odom'),
        DeclareLaunchArgument('base_frame_id', default_value='robot1_tf/base_link'),
        DeclareLaunchArgument('global_frame_id', default_value='robot1_tf/odom'),
        DeclareLaunchArgument('odom_topic', default_value='/robot1/odom'),
        DeclareLaunchArgument('laser_topic', default_value='/robot1/scan'),

        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='move_base',
            output='screen',
            parameters=[
                os.path.join(config_directory, 'costmap_common_params.yaml'),
                os.path.join(config_directory, 'local_costmap_params.yaml'),
                os.path.join(config_directory, 'global_costmap_params.yaml'),
                os.path.join(config_directory, 'move_base_params.yaml'),
                {
                    'global_costmap.global_frame': LaunchConfiguration('global_frame_id'),
                    'global_costmap.robot_base_frame': LaunchConfiguration('base_frame_id'),
                    'local_costmap.global_frame': LaunchConfiguration('odom_frame_id'),
                    'local_costmap.robot_base_frame': LaunchConfiguration('base_frame_id')
                }
            ],
            remappings=[
                ('cmd_vel', '/robot1/cmd_vel'),
                ('odom', LaunchConfiguration('odom_topic')),
                ('scan', LaunchConfiguration('laser_topic')),
                ('map', '/map'),
                ('/move_base/TebLocalPlannerROS/global_plan', '/robot1/move_base/TebLocalPlannerROS/global_plan'),
                ('/move_base/TebLocalPlannerROS/local_plan', '/robot1/move_base/TebLocalPlannerROS/local_plan'),
                ('/move_base/TebLocalPlannerROS/teb_markers', '/robot1/move_base/TebLocalPlannerROS/teb_markers'),
                ('/move_base/TebLocalPlannerROS/teb_markers_array', '/robot1/move_base/TebLocalPlannerROS/teb_markers_array'),
                ('/move_base/TebLocalPlannerROS/teb_poses', '/robot1/move_base/TebLocalPlannerROS/teb_poses'),
                ('/move_base/global_costmap/costmap', '/robot1/move_base/global_costmap/costmap'),
                ('/move_base/global_costmap/costmap_updates', '/robot1/move_base/global_costmap/costmap_updates'),
                ('/move_base/local_costmap/costmap', '/robot1/move_base/local_costmap/costmap'),
                ('/move_base/local_costmap/costmap_updates', '/robot1/move_base/local_costmap/costmap_updates'),
                ('/move_base/local_costmap/footprint', '/robot1/move_base/local_costmap/footprint'),
                ('/move_base/GlobalPlanner/parameter_descriptions', '/robot1/move_base/GlobalPlanner/parameter_descriptions'),
                ('/move_base/GlobalPlanner/parameter_updates', '/robot1/move_base/GlobalPlanner/parameter_updates'),
                ('/move_base/GlobalPlanner/plan', '/robot1/move_base/GlobalPlanner/plan'),
                ('/move_base/GlobalPlanner/potential', '/robot1/move_base/GlobalPlanner/potential'),
                ('/move_base/TebLocalPlannerROS/obstacles', '/robot1/move_base/TebLocalPlannerROS/obstacles'),
                ('/move_base/TebLocalPlannerROS/parameter_descriptions', '/robot1/move_base/TebLocalPlannerROS/parameter_descriptions'),
                ('/move_base/TebLocalPlannerROS/parameter_updates', '/robot1/move_base/TebLocalPlannerROS/parameter_updates'),
                ('/move_base/cancel', '/robot1/move_base/cancel'),
                ('/move_base/current_goal', '/robot1/move_base/current_goal'),
                ('/move_base/feedback', '/robot1/move_base/feedback'),
                ('/move_base/global_costmap/footprint', '/robot1/move_base/global_costmap/footprint'),
                ('/move_base/global_costmap/inflation_layer/parameter_descriptions', '/robot1/move_base/global_costmap/inflation_layer/parameter_descriptions'),
                ('/move_base/global_costmap/inflation_layer/parameter_updates', '/robot1/move_base/global_costmap/inflation_layer/parameter_updates'),
                ('/move_base/global_costmap/obstacle_layer/clearing_endpoints', '/robot1/move_base/global_costmap/obstacle_layer/clearing_endpoints'),
                ('/move_base/global_costmap/obstacle_layer/parameter_descriptions', '/robot1/move_base/global_costmap/obstacle_layer/parameter_descriptions'),
                ('/move_base/global_costmap/obstacle_layer/parameter_updates', '/robot1/move_base/global_costmap/obstacle_layer/parameter_updates'),
                ('/move_base/global_costmap/parameter_descriptions', '/robot1/move_base/parameter_descriptions'),
                ('/move_base/global_costmap/parameter_updates', '/robot1/move_base/global_costmap/parameter_updates'),
                ('/move_base/global_costmap/static_layer/parameter_descriptions', '/robot1/move_base/global_costmap/static_layer/parameter_descriptions'),
                ('/move_base/global_costmap/static_layer/parameter_updates', '/robot1/move_base/global_costmap/static_layer/parameter_updates'),
                ('/move_base/goal', '/robot1/move_base/goal'),
                ('/move_base/local_costmap/obstacle_layer/clearing_endpoints', '/robot1/move_base/local_costmap/obstacle_layer/clearing_endpoints'),
                ('/move_base/local_costmap/obstacle_layer/parameter_descriptions', '/robot1/move_base/local_costmap/obstacle_layer/parameter_descriptions'),
                ('/move_base/local_costmap/obstacle_layer/parameter_updates', '/robot1/move_base/local_costmap/obstacle_layer/parameter_updates'),
                ('/move_base/local_costmap/parameter_descriptions', '/robot1/move_base/local_costmap/parameter_descriptions'),
                ('/move_base/local_costmap/parameter_updates', '/robot1/move_base/local_costmap/parameter_updates'),
                ('/move_base/local_costmap/static_layer/parameter_descriptions', '/robot1/move_base/local_costmap/static_layer/parameter_descriptions'),
                ('/move_base/local_costmap/static_layer/parameter_updates', '/robot1/move_base/local_costmap/static_layer/parameter_updates'),
                ('/move_base/parameter_descriptions', '/robot1/move_base/parameter_descriptions'),
                ('/move_base/parameter_updates', '/robot1/move_base/parameter_updates'),
                ('/move_base/result', '/robot1/move_base/result'),
                ('/move_base/status', '/robot1/move_base/status'),
                ('/move_base_simple/goal', '/robot1/move_base_simple/goal'),
                ('/move_base/cancel', '/robot1/move_base/cancel'),
                ('/move_base/feedback', '/robot1/move_base/feedback'),
                ('/move_base/goal', '/robot1/move_base/goal'),
                ('/move_base/result', '/robot1/move_base/result'),
                ('/move_base/status', '/robot1/move_base/status'),
                ('/move_base/NavfnROS/plan', '/robot1/move_base/NavfnROS/plan'),
                ('/move_base/TrajectoryPlannerROS/cost_cloud', '/robot1/move_base/TrajectoryPlannerROS/cost_cloud'),
                ('/move_base/TrajectoryPlannerROS/global_plan', '/robot1/move_base/TrajectoryPlannerROS/global_plan'),
                ('/move_base/TrajectoryPlannerROS/local_plan', '/robot1/move_base/TrajectoryPlannerROS/local_plan'),
                ('/move_base/TrajectoryPlannerROS/parameter_descriptions', '/robot1/move_base/TrajectoryPlannerROS/parameter_descriptions'),
                ('/move_base/TrajectoryPlannerROS/parameter_updates', '/robot1/move_base/TrajectoryPlannerROS/parameter_updates'),
                ('/move_base/local_costmap/inflation_layer/parameter_descriptions', '/robot1/move_base/local_costmap/inflation_layer/parameter_descriptions'),
                ('/move_base/local_costmap/inflation_layer/parameter_updates', '/robot1/move_base/local_costmap/inflation_layer/parameter_updates'),
                ('/move_base/DWAPlannerROS/cost_cloud', '/robot1/move_base/DWAPlannerROS/cost_cloud'),
                ('/move_base/DWAPlannerROS/global_plan', '/robot1/move_base/DWAPlannerROS/global_plan'),
                ('/move_base/DWAPlannerROS/local_plan', '/robot1/move_base/DWAPlannerROS/local_plan'),
                ('/move_base/DWAPlannerROS/parameter_descriptions', '/robot1/move_base/DWAPlannerROS/parameter_descriptions'),
                ('/move_base/DWAPlannerROS/parameter_updates', '/robot1/move_base/DWAPlannerROS/parameter_updates'),
                ('/move_base/DWAPlannerROS/trajectory_cloud', '/robot1/move_base/DWAPlannerROS/trajectory_cloud'),
                ('/move_base/global_costmap/obstacle_layer/voxel_grid', '/robot1/move_base/global_costmap/obstacle_layer/voxel_grid'),
                ('/move_base/local_costmap/obstacle_layer/voxel_grid', '/robot1/move_base/local_costmap/obstacle_layer/voxel_grid'),
            ]
        ),
    ])

