from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    shared_dir = get_package_share_directory("perception_2d")
    config = os.path.join(shared_dir, "params", "perception_2d.yaml")

    shared_dir = get_package_share_directory("deepstream_manager")
    yolov5_env = os.environ
    yolov5_env["LD_PRELOAD"] = os.path.join(
        shared_dir, "engine/library", "libyolov5plugins.so"
    )

    return LaunchDescription(
        [
            Node(
                package="perception_2d",
                node_executable="perception_2d",
                parameters=[config],
                env=yolov5_env,
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
