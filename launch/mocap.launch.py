from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("mocap_ros"),
        "config",
        "bodies.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the mocap_ros YAML configuration file.",
        ),
        Node(
            package="mocap_ros",
            executable="mocap_node",
            name="mocap_node",
            output="screen",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
