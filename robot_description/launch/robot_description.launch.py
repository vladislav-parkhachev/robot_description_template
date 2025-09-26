import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

PKG_NAME = "robot_description"
PKG_SHARE = get_package_share_directory(PKG_NAME)

DEFAULT_MODEL_PATH = os.path.join(PKG_SHARE, "urdf", "robot.urdf.xacro")
DEFAULT_CONFIG_PATH = os.path.join(PKG_SHARE, "config", "robot_components.yaml")

DEFAULT_JSP_GUI = "false"         
DEFAULT_USE_SIM_TIME = "false"

def generate_launch_description():
    # Load YAML parameters
    with open(DEFAULT_CONFIG_PATH, "r") as f:
        yaml_params = yaml.safe_load(f)

    declared_arguments = [
        DeclareLaunchArgument(
            "model",
            default_value=DEFAULT_MODEL_PATH,
            description="Absolute path to robot URDF/Xacro file"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=DEFAULT_USE_SIM_TIME,
            description="Use simulation clock if true"
        ),
        DeclareLaunchArgument(
            "jsp_gui",
            default_value=DEFAULT_JSP_GUI,
            choices=["true", "false"],
            description="Launch joint_state_publisher_gui if true"
        ),
    ]

    # Add launch arguments from YAML
    for param, value in yaml_params.items():
        declared_arguments.append(
            DeclareLaunchArgument(
                param,
                default_value=str(value),
                description=f"Xacro parameter: {param}"
            )
        )

    # Build xacro command with substitutions
    xacro_command = ["xacro ", LaunchConfiguration("model")]
    for param in yaml_params.keys():
        xacro_command.extend([f" {param}:=", LaunchConfiguration(param)])

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(Command(xacro_command), value_type=str),
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )

    # Joint State Publisher (обычный)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("jsp_gui")),
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("jsp_gui")),
    )


    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
    ])