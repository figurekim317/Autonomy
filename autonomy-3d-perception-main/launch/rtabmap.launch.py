from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


from launch_ros.actions import Node
import yaml
import os


stereo_dir = get_package_share_directory("3d_perception")


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="3d_perception",
                node_name="point_rotation",
                node_namespace="depth_fl",
                node_executable="point_rotation",
                parameters=[stereo_dir + "/config/point_rotation_L_parameter.yaml"],
                output="screen",
            ),
            Node(
                package="3d_perception",
                node_name="point_rotation",
                node_namespace="depth_fr",
                node_executable="point_rotation",
                parameters=[stereo_dir + "/config/point_rotation_R_parameter.yaml"],
                output="screen",
            ),
            Node(
                package="rtabmap_ros",
                node_name="obstacles_detection",
                node_executable="obstacles_detection",
                node_namespace="depth_fl",
                remappings=[("cloud", "cloudPCL")],
                parameters=[
                    {"frame_id": "depth_fl"},
                    {"map_frame_id": "depth_fl"},
                    {"Grid/ClusterRadius": "0.2"},
                    {"Grid/MaxGroundAngle": "30.0"},
                    {"Grid/MinClusterSize": "3"},
                    {"Grid/MaxObstacleHeight": "3.3"},
                ],
                output="screen",
            ),
            Node(
                package="rtabmap_ros",
                node_name="obstacles_detection",
                node_executable="obstacles_detection",
                node_namespace="depth_fr",
                remappings=[("cloud", "cloudPCL")],
                parameters=[
                    {"frame_id": "depth_fr"},
                    {"map_frame_id": "depth_fr"},
                    {"Grid/ClusterRadius": "0.2"},
                    {"Grid/MaxGroundAngle": "30.0"},
                    {"Grid/MinClusterSize": "3"},
                    {"Grid/MaxObstacleHeight": "3.3"},
                ],
                output="screen",
            ),
        ]
    )
