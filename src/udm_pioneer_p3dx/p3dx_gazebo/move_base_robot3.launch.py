from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('no_static_map', default_value='true'),
        DeclareLaunchArgument('odom_frame_id', default_value='robot3_tf/odom'),
        DeclareLaunchArgument('base_frame_id', default_value='robot3_tf/base_link'),
        DeclareLaunchArgument('global_frame_id', default_value='robot3_tf/odom'),
        DeclareLaunchArgument('odom_topic', default_value='/robot3/odom'),
        DeclareLaunchArgument('laser_topic', default_value='/robot3/scan'),
        
        Node(
            package='move_base',
            executable='move_base',
            name='move_base3',
            output='screen',
            parameters=[
                {'global_costmap.global_frame': LaunchConfiguration('global_frame_id')},
                {'global_costmap.robot_base_frame': LaunchConfiguration('base_frame_id')},
                {'local_costmap.global_frame': LaunchConfiguration('odom_frame_id')},
                {'local_costmap.robot_base_frame': LaunchConfiguration('base_frame_id')}
            ],
            remappings=[
                ('cmd_vel', '/robot3/cmd_vel'),
                ('odom', LaunchConfiguration('odom_topic')),
                ('scan', LaunchConfiguration('laser_topic')),
                ('map', '/map'),
                ('/move_base3/TebLocalPlannerROS/global_plan', '/robot3/move_base/TebLocalPlannerROS/global_plan'),
                ('/move_base3/TebLocalPlannerROS/local_plan', '/robot3/move_base/TebLocalPlannerROS/local_plan'),
                ('/move_base3/TebLocalPlannerROS/teb_markers', '/robot3/move_base/TebLocalPlannerROS/teb_markers'),
                ('/move_base3/TebLocalPlannerROS/teb_markers_array', '/robot3/move_base/TebLocalPlannerROS/teb_markers_array'),
                ('/move_base3/TebLocalPlannerROS/teb_poses', '/robot3/move_base/TebLocalPlannerROS/teb_poses'),
                ('/move_base3/global_costmap/costmap', '/robot3/move_base/global_costmap/costmap'),
                ('/move_base3/global_costmap/costmap_updates', '/robot3/move_base/global_costmap/costmap_updates'),
                ('/move_base3/local_costmap/costmap', '/robot3/move_base/local_costmap/costmap'),
                ('/move_base3/local_costmap/costmap_updates', '/robot3/move_base/local_costmap/costmap_updates'),
                ('/move_base3/local_costmap/footprint', '/robot3/move_base/local_costmap/footprint'),
                ('/move_base3/GlobalPlanner/parameter_descriptions', '/robot3/move_base/GlobalPlanner/parameter_descriptions'),
                ('/move_base3/GlobalPlanner/parameter_updates', '/robot3/move_base/GlobalPlanner/parameter_updates'),
                ('/move_base3/GlobalPlanner/plan', '/robot3/move_base/GlobalPlanner/plan'),
                ('/move_base3/GlobalPlanner/potential', '/robot3/move_base/GlobalPlanner/potential'),
                ('/move_base3/TebLocalPlannerROS/obstacles', '/robot3/move_base/TebLocalPlannerROS/obstacles'),
                ('/move_base3/TebLocalPlannerROS/parameter_descriptions', '/robot3/move_base/TebLocalPlannerROS/parameter_descriptions'),
                ('/move_base3/TebLocalPlannerROS/parameter_updates', '/robot3/move_base/TebLocalPlannerROS/parameter_updates'),
                ('/move_base3/cancel', '/robot3/move_base/cancel'),
                ('/move_base3/current_goal', '/robot3/move_base/current_goal'),
                ('/move_base3/feedback', '/robot3/move_base/feedback'),
                ('/move_base3/global_costmap/footprint', '/robot3/move_base/global_costmap/footprint'),
                ('/move_base3/global_costmap/inflation_layer/parameter_descriptions', '/robot3/move_base/global_costmap/inflation_layer/parameter_descriptions'),
                ('/move_base3/global_costmap/inflation_layer/parameter_updates', '/robot3/move_base/global_costmap/inflation_layer/parameter_updates'),
                ('/move_base3/global_costmap/obstacle_layer/clearing_endpoints', '/robot3/move_base/global_costmap/obstacle_layer/clearing_endpoints'),
                ('/move_base3/global_costmap/obstacle_layer/parameter_descriptions', '/robot3/move_base/global_costmap/obstacle_layer/parameter_descriptions'),
                ('/move_base3/global_costmap/obstacle_layer/parameter_updates', '/robot3/move_base/global_costmap/obstacle_layer/parameter_updates'),
                ('/move_base3/global_costmap/parameter_descriptions', '/robot3/move_base/parameter_descriptions'),
                ('/move_base3/global_costmap/parameter_updates', '/robot3/move_base/global_costmap/parameter_updates'),
                ('/move_base3/global_costmap/static_layer/parameter_descriptions', '/robot3/move_base/global_costmap/static_layer/parameter_descriptions'),
                ('/move_base3/global_costmap/static_layer/parameter_updates', '/robot3/move_base/global_costmap/static_layer/parameter_updates'),
                ('/move_base3/goal', '/robot3/move_base/goal'),
                ('/move_base3/local_costmap/obstacle_layer/clearing_endpoints', '/robot3/move_base/local_costmap/obstacle_layer/clearing_endpoints'),
                ('/move_base3/local_costmap/obstacle_layer/parameter_descriptions', '/robot3/move_base/local_costmap/obstacle_layer/parameter_descriptions'),
                ('/move_base3/local_costmap/obstacle_layer/parameter_updates', '/robot3/move_base/local_costmap/obstacle_layer/parameter_updates'),
                ('/move_base3/local_costmap/parameter_descriptions', '/robot3/move_base/local_costmap/parameter_descriptions'),
                ('/move_base3/local_costmap/parameter_updates', '/robot3/move_base/local_costmap/parameter_updates'),
                ('/move_base3/local_costmap/static_layer/parameter_descriptions', '/robot3/move_base/local_costmap/static_layer/parameter_descriptions'),
                ('/move_base3/local_costmap/static_layer/parameter_updates', '/robot3/move_base/local_costmap/static_layer/parameter_updates'),
                ('/move_base3/parameter_descriptions', '/robot3/move_base/parameter_descriptions'),
                ('/move_base3/parameter_updates', '/robot3/move_base/parameter_updates'),
                ('/move_base3/result', '/robot3/move_base/result'),
                ('/move_base3/status', '/robot3/move_base/status'),
                ('/move_base_simple/goal', '/robot3/move_base_simple/goal'),
                ('/move_base/cancel', '/robot3/move_base/cancel'),
                ('/move_base/feedback', '/robot3/move_base/feedback'),
                ('/move_base/goal', '/robot3/move_base/goal'),
                ('/move_base/result', '/robot3/move_base/result'),
                ('/move_base/status', '/robot3/move_base/status'),
                ('/move_base/NavfnROS/plan', '/robot3/move_base/NavfnROS/plan'),
                ('/move_base/TrajectoryPlannerROS/cost_cloud', '/robot3/move_base/TrajectoryPlannerROS/cost_cloud'),
                ('/move_base/TrajectoryPlannerROS/global_plan', '/robot3/move_base/TrajectoryPlannerROS/global_plan'),
                ('/move_base/TrajectoryPlannerROS/local_plan', '/robot3/move_base/TrajectoryPlannerROS/local_plan'),
                ('/move_base/TrajectoryPlannerROS/parameter_descriptions', '/robot3/move_base/TrajectoryPlannerROS/parameter_descriptions'),
                ('/move_base/TrajectoryPlannerROS/parameter_updates', '/robot3/move_base/TrajectoryPlannerROS/parameter_updates'),
                ('/move_base/local_costmap/inflation_layer/parameter_descriptions', '/robot3/move_base/local_costmap/inflation_layer/parameter_descriptions'),
                ('/move_base/local_costmap/inflation_layer/parameter_updates', '/robot3/move_base/local_costmap/inflation_layer/parameter_updates'),
                ('/move_base/DWAPlannerROS/cost_cloud', '/robot3/move_base/DWAPlannerROS/cost_cloud'),
                ('/move_base/DWAPlannerROS/global_plan', '/robot3/move_base/DWAPlannerROS/global_plan'),
                ('/move_base/DWAPlannerROS/local_plan', '/robot3/move_base/DWAPlannerROS/local_plan'),
                ('/move_base/DWAPlannerROS/parameter_descriptions', '/robot3/move_base/DWAPlannerROS/parameter_descriptions'),
                ('/move_base/DWAPlannerROS/parameter_updates', '/robot3/move_base/DWAPlannerROS/parameter_updates'),
                ('/move_base/DWAPlannerROS/trajectory_cloud', '/robot3/move_base/DWAPlannerROS/trajectory_cloud'),
                ('/move_base/global_costmap/obstacle_layer/voxel_grid', '/robot3/move_base/global_costmap/obstacle_layer/voxel_grid'),
                ('/move_base/local_costmap/obstacle_layer/voxel_grid', '/robot3/move_base/local_costmap/obstacle_layer/voxel_grid')
            ]
        )
    ])