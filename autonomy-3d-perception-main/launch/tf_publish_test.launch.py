from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import yaml
import os
import math
import numpy as np

stereo_dir = get_package_share_directory("3d_perception")


def quaternion_rotation_matrix(Q):
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 1 - 2 * (q1 * q1 + q2 * q2)
    r01 = 2 * (q0 * q1 - q2 * q3)
    r02 = 2 * (q0 * q2 + q3 * q1)

    # Second row of the rotation matrix
    r10 = 2 * (q0 * q1 + q2 * q3)
    r11 = 1 - 2 * (q0 * q0 + q2 * q2)
    r12 = 2 * (q1 * q2 - q3 * q0)

    # Third row of the rotation matrix
    r20 = 2 * (q0 * q2 - q3 * q1)
    r21 = 2 * (q1 * q2 + q3 * q0)
    r22 = 1 - 2 * (q0 * q0 + q1 * q1)

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

    return rot_matrix


def rotation2euler(R):

    r11 = R[0][0]
    r12 = R[0][1]
    r13 = R[0][2]
    r21 = R[1][0]
    r22 = R[1][1]
    r23 = R[1][2]
    r31 = R[2][0]
    r32 = R[2][1]
    r33 = R[2][2]

    roll_x = math.atan2(r32, r33)
    pitch_y = math.asin(-r31)
    yaw_z = math.atan2(r21, r11)

    return roll_x, pitch_y, yaw_z  # in radians


def rotation2euler_forPCL(R):

    r11 = R[0][0]
    r12 = R[0][1]
    r13 = R[0][2]
    r21 = R[1][0]
    r22 = R[1][1]
    r23 = R[1][2]
    r31 = R[2][0]
    r32 = R[2][1]
    r33 = R[2][2]

    roll_x = math.atan2(r32, r22)
    pitch_y = math.atan2(r13, r11)
    yaw_z = math.asin(-r12)

    return roll_x, pitch_y, yaw_z  # in radians


with open(os.path.expanduser("~/robot_cfg/depth_extrinsic_params.yaml")) as f:
    conf = yaml.load(f)

param_list = conf["depth_extrinsic"]

q_f_l = param_list["left_quaternion"]
q_f_r = param_list["right_quaternion"]
t_f_l = param_list["left_translation"]
t_f_r = param_list["right_translation"]
base_link_xyz = param_list["base_link_xyz"]

R_fl = quaternion_rotation_matrix(q_f_l)
R_fl = R_fl.T
E_fl_pcl = rotation2euler_forPCL(R_fl)
E_fl = rotation2euler(R_fl)

R_fr = quaternion_rotation_matrix(q_f_r)
E_fr_pcl = rotation2euler_forPCL(R_fr)
E_fr = rotation2euler(R_fr)

t_f_r = np.dot(R_fr, t_f_r)

print(E_fl_pcl)
print(E_fl)
print(E_fr_pcl)
print(E_fr)
print(t_f_l)
print(t_f_r)


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                node_executable="static_transform_publisher",
                node_name="rs_base_link",
                arguments=[
                    str(base_link_xyz[0]),
                    str(base_link_xyz[1]),
                    str(base_link_xyz[2]),
                    "-1.5708",
                    "0",
                    "-1.5708",
                    "base_link",
                    "mono_F_Link",
                ],
            ),
            Node(
                package="tf2_ros",
                node_executable="static_transform_publisher",
                node_name="rs_base_link",
                arguments=[
                    str(base_link_xyz[0]),
                    str(base_link_xyz[1]),
                    str(base_link_xyz[2]),
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "base_link_pcl",
                ],
            ),
            Node(
                package="tf2_ros",
                node_executable="static_transform_publisher",
                node_name="rs_left_depth_frame",
                arguments=[
                    str(t_f_l[0]),
                    str(t_f_l[1]),
                    str(t_f_l[2]),
                    str(E_fl[2]),
                    str(E_fl[1]),
                    str(E_fl[0]),
                    "mono_F_Link",
                    "left_depth_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                node_executable="static_transform_publisher",
                node_name="rs_right_depth_frame",
                arguments=[
                    str(-t_f_r[0]),
                    str(-t_f_r[1]),
                    str(-t_f_r[2]),
                    str(E_fr[2]),
                    str(E_fr[1]),
                    str(E_fr[0]),
                    "mono_F_Link",
                    "right_depth_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                node_executable="static_transform_publisher",
                node_name="rs_depth_link2_1",
                arguments=[
                    str(t_f_l[2]),
                    str(-t_f_l[0]),
                    str(-t_f_l[1]),
                    str(-E_fl[1]),
                    "0",
                    "0",
                    "base_link_pcl",
                    "depth_fl",
                ],
            ),
            Node(
                package="tf2_ros",
                node_executable="static_transform_publisher",
                node_name="rs_depth_link2_2",
                arguments=[
                    str(-t_f_r[2]),
                    str(t_f_r[0]),
                    str(t_f_r[1]),
                    str(-E_fr[1]),
                    "0",
                    "0",
                    "base_link_pcl",
                    "depth_fr",
                ],
            ),
            Node(
                package="tf2_ros",
                node_executable="static_transform_publisher",
                node_name="ti_link2_2",
                arguments=[
                    "0.04",
                    "0",
                    "-0.06",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "ti_mmwave",
                ],
            ),
            Node(
                package="3d_perception",
                node_executable="range_region",
                node_name="range_region",
            ),
        ]
    )
