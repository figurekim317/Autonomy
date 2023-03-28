from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():

    OD_file_path = get_package_share_directory("3d_perception")
    depth_file_path = get_package_share_directory("3d_perception")
    curb_file_path = get_package_share_directory("3d_perception")
    seg_fusion_file_path = get_package_share_directory("3d_perception")
    tf_file_path = get_package_share_directory("3d_perception")

    OD_sys = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            OD_file_path + "/launch/obstacles_detection.launch.py"
        )
    )

    depth_sys = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_file_path + "/launch/rtabmap.launch.py")
    )

    curb_sys = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            curb_file_path + "/launch/curb_detection.launch.py"
        )
    )

    seg_sys = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            seg_fusion_file_path + "/launch/seg_fusion.launch.py"
        )
    )

    tf_sys = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            tf_file_path + "/launch/tf_publish_test.launch.py"
        )
    )

    return LaunchDescription(
        [
            OD_sys,
            depth_sys,
            curb_sys,
            seg_sys,
            # tf_sys,
        ]
    )
