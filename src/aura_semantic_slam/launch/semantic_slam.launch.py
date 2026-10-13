"""Complete AURA semantic RGB-D SLAM with external odometry and YOLO."""
from datetime import datetime
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("aura_semantic_slam"))
    executables = Path(get_package_prefix("aura_semantic_slam"))/"lib/aura_semantic_slam"
    results = Path.home()/"ai_envs/project/c++_project/rtabmap_results"
    results.mkdir(parents=True, exist_ok=True)
    database = str(results/datetime.now().strftime("semantic_%Y%m%d_%H%M%S_%f.db"))
    adapter = ExecuteProcess(cmd=["/usr/bin/python3", str(executables/"rgbd_adapter.py"),
                         "--ros-args", "-p", "use_sim_time:=true", "-p",
                         ["odom_input_topic:=", LaunchConfiguration("odom_topic")]], output="screen")
    stock = os.path.join(get_package_share_directory("rtabmap_launch"), "launch", "rtabmap.launch.py")
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(stock), launch_arguments={
        "namespace": "aura_slam", "use_sim_time": "true", "frame_id": "aura_slam_base",
        "map_frame_id": "aura_slam_map", "odom_frame_id": "", "odom_topic": "/aura_slam/odom",
        "visual_odometry": "false", "icp_odometry": "false", "publish_tf_map": "true",
        "odom_sensor_sync": "true", "rgb_topic": "/aura_slam/rgb/image",
        "depth_topic": "/aura_slam/depth/image", "camera_info_topic": "/aura_slam/rgb/camera_info",
        "rgbd_sync": "true", "approx_rgbd_sync": "true", "approx_sync": "true",
        "approx_sync_max_interval": "0.05", "topic_queue_size": "30", "sync_queue_size": "30",
        "qos": "2", "qos_odom": "1", "compressed": "false",
        "rgb_image_transport": "raw", "depth_image_transport": "raw",
        "rtabmap_viz": LaunchConfiguration("rtabmap_gui"), "rviz": "false", "database_path": database,
        "args": "--Rtabmap/DetectionRate 1.0 --Reg/Force3DoF true",
    }.items())
    mapper = ExecuteProcess(cmd=[LaunchConfiguration("mapper_python"),
                         str(executables/"semantic_slam_mapper.py"), "--ros-args",
                         "--params-file", LaunchConfiguration("config")], output="screen")
    rviz = Node(package="rviz2", executable="rviz2", name="aura_semantic_rviz",
                arguments=["-d", str(share/"rviz/semantic_slam.rviz")],
                parameters=[{"use_sim_time": True}], output="screen",
                condition=IfCondition(LaunchConfiguration("show_rviz")))
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/aura/odom_fused",
                              description="Input pose source: /aura/odom_fused or /odom"),
        DeclareLaunchArgument("mapper_python", default_value=str(Path.home()/"ai_envs/robotics_ai/bin/python"),
                              description="Python interpreter containing YOLO and ROS imports"),
        DeclareLaunchArgument("config", default_value=str(share/"config/semantic_slam.yaml")),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        DeclareLaunchArgument("rtabmap_gui", default_value="false"),
        LogInfo(msg=["AURA semantic SLAM input: ", LaunchConfiguration("odom_topic")]),
        LogInfo(msg=f"RTAB-Map database: {database}"),
        RegisterEventHandler(OnProcessExit(target_action=adapter,
              on_exit=[EmitEvent(event=Shutdown(reason="RGB-D adapter exited"))])),
        RegisterEventHandler(OnProcessExit(target_action=mapper,
              on_exit=[EmitEvent(event=Shutdown(reason="Semantic mapper exited; inspect its error above"))])),
        adapter, slam, mapper, rviz,
    ])
