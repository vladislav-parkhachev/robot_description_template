import os
import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG_NAME = "robot_description"
PKG_SHARE = get_package_share_directory(PKG_NAME)

# DEFAULT_RVIZ_CONFIG = os.path.join(PKG_SHARE, "config", "robot_view.rviz")

PKG_CONFIG = os.path.join(PKG_SHARE, "config", "robot_view.rviz")
LOCAL_CONFIG = "/override_config/robot_view.rviz"

DEFAULT_JSP_GUI = "true"         
DEFAULT_USE_SIM_TIME = "false"
DEFAULT_RVIZ = "true"


def _resolve_rviz_config():
    """Ensure RViz uses a host-mounted config so edits persist outside Docker."""
    local_dir = os.path.dirname(LOCAL_CONFIG)
    try:
        os.makedirs(local_dir, exist_ok=True)
        if not os.path.isfile(LOCAL_CONFIG):
            shutil.copyfile(PKG_CONFIG, LOCAL_CONFIG)
        return LOCAL_CONFIG
    except OSError as exc:
        print(f"[robot_view.launch] Using packaged RViz config: {exc}")
        return PKG_CONFIG


def generate_launch_description():
    default_rviz_config = _resolve_rviz_config()

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value=DEFAULT_USE_SIM_TIME),
        DeclareLaunchArgument("jsp_gui", default_value=DEFAULT_JSP_GUI),
        DeclareLaunchArgument("rviz", default_value=DEFAULT_RVIZ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config file",
        ),
    ]

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([PKG_SHARE, "launch", "robot_description.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "jsp_gui": LaunchConfiguration("jsp_gui"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(declared_arguments + [
        robot_description_launch,
        rviz_node,
    ])
