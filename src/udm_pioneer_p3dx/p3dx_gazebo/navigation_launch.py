
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['robot2/controller_server',
                       'robot2/planner_server',
                       'robot2/recoveries_server',
                       'robot2/bt_navigator',
                    #    'robot2/waypoint_follower'
                       ]

    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'default_bt_xml_filename': default_bt_xml_filename,
    #     'autostart': autostart,
    #     'map_subscribe_transient_local': map_subscribe_transient_local}

    # configured_params = RewrittenYaml(
    #         source_file=params_file,
    #         root_key=namespace,
    #         param_rewrites=param_substitutions,
    #         convert_types=True)

    controller_yaml = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'controller.yaml')
    planner_yaml = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'planner.yaml')
    recoveries_yaml = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'recoveries.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'bt_navigator.yaml')
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='robot2',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('udm_pioneer_p3dx'), 'p3dx_navigation', 'config', 'plswork.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description='Whether to set the map subscriber QoS to transient local'),

        Node(
            namespace="robot2",
            name="controller_server",
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': True}],
        ),

        Node(
            namespace="robot2",
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': True}]),

        Node(
            namespace="robot2",
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recoveries_yaml, {'use_sim_time': True}],
),

        Node(
            namespace="robot2",
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': True}],
        ),

#         Node(
#             package='nav2_waypoint_follower',
#             executable='waypoint_follower',
#             name='waypoint_follower',
#             output='screen',
#             parameters=[{'params_file': '/home/ara/main_ws/src/udm_pioneer_p3dx/p3dx_navigation/config/plswork.yaml', 'use_sim_time': True}],
#            ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            # namespace="robot2",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout': 0.0},
                        {'node_names': lifecycle_nodes}]),

    ])