"""Separate six-object scene with a raised camera; the library launch is preserved."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("description_new")
    sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    description = ParameterValue(Command([
        "xacro ", os.path.join(share, "urdf", "robot_mapping_demo.urdf.xacro"),
        " use_ros2_control:=false sim_mode:=true"]), value_type=str)
    resources = os.pathsep.join(filter(None, (
        os.path.join(share, "worlds", "aura_mapping_demo_assets"),
        os.path.dirname(share), os.path.join(share, "worlds"),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""))))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=os.path.join(share, "worlds", "aura_mapping_demo.sdf")),
        DeclareLaunchArgument("spawn_x", default_value="-3.0"),
        DeclareLaunchArgument("spawn_y", default_value="-2.2"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resources),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": sim}], output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": ["-r -v4 ", LaunchConfiguration("world")],
                              "on_exit_shutdown": "true"}.items()),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "hv_bot",
                        "-x", LaunchConfiguration("spawn_x"), "-y", LaunchConfiguration("spawn_y"),
                        "-z", "0.2", "-R", "0", "-P", "0", "-Y", LaunchConfiguration("spawn_yaw")],
             parameters=[{"use_sim_time": sim}]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(share, "config", "gz_bridge.yaml"),
                          "use_sim_time": sim}]),
    ])
