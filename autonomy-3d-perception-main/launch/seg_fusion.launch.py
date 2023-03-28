from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import yaml
import os


def generate_launch_description():

    stereo_dir = get_package_share_directory("3d_perception")
    moudle_dir = get_package_share_directory("perception_module")

    return LaunchDescription(
        [
            Node(
                package="3d_perception",
                node_name="combine_ground",
                node_executable="combine_ground",
                parameters=[
                    stereo_dir + "/config/combine_ground_for_seg_fusion_parameter.yaml"
                ],
                output="screen",
            ),
            Node(
                package="perception_module",
                node_name="segmentation_fusion",
                node_executable="segmentation_fusion",
                parameters=[moudle_dir + "/config/segmentation_fusion_parameter.yaml"],
                output="screen",
            ),
        ]
    )
