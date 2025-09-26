import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

PKG_NAME = "robot_description"
PKG_SHARE = get_package_share_directory(PKG_NAME)

DEFAULT_MODEL_PATH = os.path.join(PKG_SHARE, "urdf", "robot.urdf.xacro")
DEFAULT_RVIZ_CONFIG_PATH = os.path.join(PKG_SHARE, "config", "robot_view.rviz")

DEFAULT_RVIZ = "true"                   # Launch RViz by default
DEFAULT_JSP_GUI = "true"                # Use GUI sliders? (false = normal joint_state_publisher)
DEFAULT_JSP = "true"                    # Launch joint_state_publisher if GUI is disabled


def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=DEFAULT_MODEL_PATH,
        description="Absolute path to robot URDF/Xacro file"
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value=DEFAULT_RVIZ,
        choices=["true", "false"],
        description="Launch RViz?"
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=DEFAULT_RVIZ_CONFIG_PATH,
        description="Absolute path to RViz config file"
    )

    jsp_gui_arg = DeclareLaunchArgument(
        "jsp_gui",
        default_value=DEFAULT_JSP_GUI,
        choices=["true", "false"],
        description="Use joint_state_publisher_gui instead of joint_state_publisher"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", LaunchConfiguration("model")]),
                value_type=str
            )
        }],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("jsp_gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("jsp_gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        rviz_config_arg,
        jsp_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
