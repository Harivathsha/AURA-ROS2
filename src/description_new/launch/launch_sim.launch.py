"""Launch the existing robot in an SDF world; default is the AURA maze."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('description_new')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    resources = os.pathsep.join(filter(None, [
        os.path.dirname(share), os.path.join(share, 'worlds'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value='pioneer3dx.rviz'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            share, 'worlds', 'aura_maze.sdf')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rsp.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'use_ros2_control': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -v4 ', LaunchConfiguration('world')],
                              'on_exit_shutdown': 'true'}.items()),
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-topic', 'robot_description', '-name', 'hv_bot',
                       '-x', '0', '-y', '0', '-z', '0.2',
                       '-R', '0.0', '-P', '0.0', '-Y', '3.14'],
            parameters=[{'use_sim_time': sim_time}]),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', output='screen',
            parameters=[{'config_file': os.path.join(share, 'config', 'gz_bridge.yaml'),
                         'use_sim_time': sim_time}]),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([
                share, 'rviz', LaunchConfiguration('rviz_config')])],
            parameters=[{'use_sim_time': sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
