from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import yaml
import os


def generate_launch_description():

    stereo_dir = get_package_share_directory("3d_perception")

    return LaunchDescription(
        [
            Node(
                package="3d_perception",
                node_name="obstacles_detection",
                node_executable="obstacles_detection",
                parameters=[stereo_dir + "/config/obstacles_detection_parameter.yaml"],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-rosout-logs",
                    "--disable-stdout-logs",
                    "--disable-external-lib-logs",
                ],
            ),
        ]
    )
