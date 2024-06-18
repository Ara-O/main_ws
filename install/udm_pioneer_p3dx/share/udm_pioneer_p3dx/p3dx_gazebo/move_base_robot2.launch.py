from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    config_directory = LaunchConfiguration('config_directory', default=os.path.join(
        get_package_share_directory('p3dx_navigation'), 'config'))
    
    return LaunchDescription([
        DeclareLaunchArgument('no_static_map', default_value='true'),
        DeclareLaunchArgument('odom_frame_id', default_value='robot2_tf/odom'),
        DeclareLaunchArgument('base_frame_id', default_value='robot2_tf/base_link'),
        DeclareLaunchArgument('global_frame_id', default_value='robot2_tf/odom'),
        DeclareLaunchArgument('odom_topic', default_value='/robot2/odom'),
        DeclareLaunchArgument('laser_topic', default_value='/robot2/scan'),

        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='move_base2',
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
                ('cmd_vel', '/robot2/cmd_vel'),
                ('odom', LaunchConfiguration('odom_topic')),
                ('scan', LaunchConfiguration('laser_topic')),
                ('map', '/map'),
                ('/move_base2/TebLocalPlannerROS/global_plan', '/robot2/move_base/TebLocalPlannerROS/global_plan'),
                ('/move_base2/TebLocalPlannerROS/local_plan', '/robot2/move_base/TebLocalPlannerROS/local_plan'),
                ('/move_base2/TebLocalPlannerROS/teb_markers', '/robot2/move_base/TebLocalPlannerROS/teb_markers'),
                ('/move_base2/TebLocalPlannerROS/teb_markers_array', '/robot2/move_base/TebLocalPlannerROS/teb_markers_array'),
                ('/move_base2/TebLocalPlannerROS/teb_poses', '/robot2/move_base/TebLocalPlannerROS/teb_poses'),
                ('/move_base2/global_costmap/costmap', '/robot2/move_base/global_costmap/costmap'),
                ('/move_base2/global_costmap/costmap_updates', '/robot2/move_base/global_costmap/costmap_updates'),
                ('/move_base2/local_costmap/costmap', '/robot2/move_base/local_costmap/costmap'),
                ('/move_base2/local_costmap/costmap_updates', '/robot2/move_base/local_costmap/costmap_updates'),
                ('/move_base2/local_costmap/footprint', '/robot2/move_base/local_costmap/footprint'),
                ('/move_base2/GlobalPlanner/parameter_descriptions', '/robot2/move_base/GlobalPlanner/parameter_descriptions'),
                ('/move_base2/GlobalPlanner/parameter_updates', '/robot2/move_base/GlobalPlanner/parameter_updates'),
                ('/move_base2/GlobalPlanner/plan', '/robot2/move_base/GlobalPlanner/plan'),
                ('/move_base2/GlobalPlanner/potential', '/robot2/move_base/GlobalPlanner/potential'),
                ('/move_base2/TebLocalPlannerROS/obstacles', '/robot2/move_base/TebLocalPlannerROS/obstacles'),
                ('/move_base2/TebLocalPlannerROS/parameter_descriptions', '/robot2/move_base/TebLocalPlannerROS/parameter_descriptions'),
                ('/move_base2/TebLocalPlannerROS/parameter_updates', '/robot2/move_base/TebLocalPlannerROS/parameter_updates'),
                ('/move_base2/cancel', '/robot2/move_base/cancel'),
                ('/move_base2/current_goal', '/robot2/move_base/current_goal'),
                ('/move_base2/feedback', '/robot2/move_base/feedback'),
                ('/move_base2/global_costmap/footprint', '/robot2/move_base/global_costmap/footprint'),
                ('/move_base2/global_costmap/inflation_layer/parameter_descriptions', '/robot2/move_base/global_costmap/inflation_layer/parameter_descriptions'),
                ('/move_base2/global_costmap/inflation_layer/parameter_updates', '/robot2/move_base/global_costmap/inflation_layer/parameter_updates'),
                ('/move_base2/global_costmap/obstacle_layer/clearing_endpoints', '/robot2/move_base/global_costmap/obstacle_layer/clearing_endpoints'),
                ('/move_base2/global_costmap/obstacle_layer/parameter_descriptions', '/robot2/move_base/global_costmap/obstacle_layer/parameter_descriptions'),
                ('/move_base2/global_costmap/obstacle_layer/parameter_updates', '/robot2/move_base/global_costmap/obstacle_layer/parameter_updates'),
                ('/move_base2/global_costmap/parameter_descriptions', '/robot2/move_base/parameter_descriptions'),
                ('/move_base2/global_costmap/parameter_updates', '/robot2/move_base/global_costmap/parameter_updates'),
                ('/move_base2/global_costmap/static_layer/parameter_descriptions', '/robot2/move_base/global_costmap/static_layer/parameter_descriptions'),
                ('/move_base2/global_costmap/static_layer/parameter_updates', '/robot2/move_base/global_costmap/static_layer/parameter_updates'),
                ('/move_base2/goal', '/robot2/move_base/goal'),
                ('/move_base2/local_costmap/obstacle_layer/clearing_endpoints', '/robot2/move_base/local_costmap/obstacle_layer/clearing_endpoints'),
                ('/move_base2/local_costmap/obstacle_layer/parameter_descriptions', '/robot2/move_base/local_costmap/obstacle_layer/parameter_descriptions'),
                ('/move_base2/local_costmap/obstacle_layer/parameter_updates', '/robot2/move_base/local_costmap/obstacle_layer/parameter_updates'),
                ('/move_base2/local_costmap/parameter_descriptions', '/robot2/move_base/local_costmap/parameter_descriptions'),
                ('/move_base2/local_costmap/parameter_updates', '/robot2/move_base/local_costmap/parameter_updates'),
                ('/move_base2/local_costmap/static_layer/parameter_descriptions', '/robot2/move_base/local_costmap/static_layer/parameter_descriptions'),
                ('/move_base2/local_costmap/static_layer/parameter_updates', '/robot2/move_base/local_costmap/static_layer/parameter_updates'),
                ('/move_base2/parameter_descriptions', '/robot2/move_base/parameter_descriptions'),
                ('/move_base2/parameter_updates', '/robot2/move_base/parameter_updates'),
                ('/move_base2/result', '/robot2/move_base/result'),
                ('/move_base2/status', '/robot2/move_base/status'),
                ('/move_base_simple/goal', '/robot2/move_base_simple/goal'),
                ('/move_base/cancel', '/robot2/move_base/cancel'),
                ('/move_base/feedback', '/robot2/move_base/feedback'),
                ('/move_base/goal', '/robot2/move_base/goal'),
                ('/move_base/result', '/robot2/move_base/result'),
                ('/move_base/status', '/robot2/move_base/status'),
                ('/move_base/NavfnROS/plan', '/robot2/move_base/NavfnROS/plan'),
                ('/move_base/TrajectoryPlannerROS/cost_cloud', '/robot2/move_base/TrajectoryPlannerROS/cost_cloud'),
                ('/move_base/TrajectoryPlannerROS/global_plan', '/robot2/move_base/TrajectoryPlannerROS/global_plan'),
                ('/move_base/TrajectoryPlannerROS/local_plan', '/robot2/move_base/TrajectoryPlannerROS/local_plan'),
                ('/move_base/TrajectoryPlannerROS/parameter_descriptions', '/robot2/move_base/TrajectoryPlannerROS/parameter_descriptions'),
                ('/move_base/TrajectoryPlannerROS/parameter_updates', '/robot2/move_base/TrajectoryPlannerROS/parameter_updates'),
                ('/move_base/local_costmap/inflation_layer/parameter_descriptions', '/robot2/move_base/local_costmap/inflation_layer/parameter_descriptions'),
                ('/move_base/local_costmap/inflation_layer/parameter_updates', '/robot2/move_base/local_costmap/inflation_layer/parameter_updates'),
                ('/move_base/DWAPlannerROS/cost_cloud', '/robot2/move_base/DWAPlannerROS/cost_cloud'),
                ('/move_base/DWAPlannerROS/global_plan', '/robot2/move_base/DWAPlannerROS/global_plan'),
                ('/move_base/DWAPlannerROS/local_plan', '/robot2/move_base/DWAPlannerROS/local_plan'),
                ('/move_base/DWAPlannerROS/parameter_descriptions', '/robot2/move_base/DWAPlannerROS/parameter_descriptions'),
                ('/move_base/DWAPlannerROS/parameter_updates', '/robot2/move_base/DWAPlannerROS/parameter_updates'),
                ('/move_base/DWAPlannerROS/trajectory_cloud', '/robot2/move_base/DWAPlannerROS/trajectory_cloud'),
                ('/move_base/global_costmap/obstacle_layer/voxel_grid', '/robot2/move_base/global_costmap/obstacle_layer/voxel_grid'),
                ('/move_base/local_costmap/obstacle_layer/voxel_grid', '/robot2/move_base/local_costmap/obstacle_layer/voxel_grid')
            ]
        )
    ])

def get_package_share_directory(package_name):
    # Define your function to get the package share directory
    pass
