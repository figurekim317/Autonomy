from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    shared_dir = get_package_share_directory("perception_2d")
    transform_config = os.path.join(shared_dir, "params", "coordinates_transform.yaml")

    return LaunchDescription(
        [
            Node(
                package="perception_module",
                node_name="detection_transformer",
                node_executable="detection_transformer",
                parameters=[transform_config],
                output="screen",
            ),
        ]
    )
