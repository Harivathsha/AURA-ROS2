"""Load the maze map, AMCL, and the Nav2 nodes used by this A* demo."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navigation = get_package_share_directory('navigation')
    bringup = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    sim_time_parameter = ParameterValue(use_sim_time, value_type=bool)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Start only configured demo servers. The stock Jazzy launch also starts
    # route and docking servers, which this maze demo does not configure.
    servers = [
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_smoother', 'smoother_server', 'smoother_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
    ]
    velocity_producers = {
        'controller_server', 'behavior_server', 'velocity_smoother'}
    navigation_nodes = [
        Node(
            package=package, executable=executable, name=name,
            output='screen',
            parameters=[params_file, {'use_sim_time': sim_time_parameter}],
            remappings=remappings + (
                [('cmd_vel', 'cmd_vel_nav')]
                if name in velocity_producers else []),
        )
        for package, executable, name in servers
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            navigation, 'maps', 'aura_maze.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            navigation, 'config', 'aura_maze_nav.yaml')),
        # Included launch arguments mutate their context. Keep AMCL's
        # params_file inside this scope so Nav2 still reads aura_maze_nav.yaml.
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    bringup, 'launch', 'localization_launch.py')),
                launch_arguments={
                    'map': map_file,
                    'params_file': os.path.join(
                        navigation, 'config', 'aura_maze_amcl.yaml'),
                    'use_sim_time': use_sim_time,
                    'autostart': 'True',
                    'use_composition': 'False',
                }.items()),
        ]),
        *navigation_nodes,
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': sim_time_parameter,
                'autostart': True,
                'node_names': [name for _, _, name in servers],
            }]),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(navigation, 'rviz', 'aura_maze.rviz')],
            parameters=[{'use_sim_time': sim_time_parameter}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
    ])
