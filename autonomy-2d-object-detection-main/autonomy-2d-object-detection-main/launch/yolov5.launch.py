from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    shared_dir = get_package_share_directory("object_detection")
    config = os.path.join(shared_dir, "params", "object_detection.yaml")
    transform_config = os.path.join(shared_dir, "params", "coordinates_transform.yaml")

    shared_dir = get_package_share_directory("deepstream_manager")
    yolov5_env = os.environ
    yolov5_env["LD_PRELOAD"] = os.path.join(
        shared_dir, "engine/library", "libyolov5plugins.so"
    )

    return LaunchDescription(
        [
            Node(
                package="object_detection",
                node_executable="object_detection",
                parameters=[config],
                env=yolov5_env,
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="perception_module",
                node_name="detection_transformer",
                node_executable="detection_transformer",
                parameters=[transform_config],
                output="screen",
            ),
        ]
    )
