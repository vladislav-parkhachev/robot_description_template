import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")

    default_model_path = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="Absolute path to robot URDF/Xacro file"
    )

    jsp_arg = DeclareLaunchArgument(
        "joint_state_publisher",
        default_value="true",
        choices=["true", "false"],
        description="Launch joint_state_publisher?"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", LaunchConfiguration("model")])
        }],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("joint_state_publisher")),
    )

    return LaunchDescription([
        model_arg,
        jsp_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])