#ifndef __RANGE_UTIL_H__
#define __RANGE_UTIL_H__

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core.hpp>

#include "autonomy_ros2_message/msg/stereo_object.hpp"

constexpr int QUEUE_SIZE = 1;

pcl::PointXYZI input_point(pcl::PointXYZI &point);
pcl::PointXYZI input_point(autonomy_ros2_message::msg::StereoPoint &point);

autonomy_ros2_message::msg::StereoPoint input_custom_point(const autonomy_ros2_message::msg::StereoPoint &point);
autonomy_ros2_message::msg::StereoPoint input_custom_point(const pcl::PointXYZI &point);

pcl::PointXYZI input_point_with_intensity(const pcl::PointXYZ &point, const float &intensity);
pcl::PointXYZI input_point_with_intensity(const pcl::PointXYZI &point, const float &intensity);
pcl::PointXYZI input_point_with_intensity(const autonomy_ros2_message::msg::StereoPoint &point, const float &intensity);

autonomy_ros2_message::msg::StereoPoint input_custom_point_with_intensity(const pcl::PointXYZI &point, const float &intensity);
autonomy_ros2_message::msg::StereoPoint input_custom_point_with_intensity(const autonomy_ros2_message::msg::StereoPoint &point, const float &intensity);

double distance(const pcl::PointXYZI &pt);
double distance(const autonomy_ros2_message::msg::StereoPoint &pt);
double distance(const autonomy_ros2_message::msg::StereoPoint &before, const autonomy_ros2_message::msg::StereoPoint &after);
double distance(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2);
double distance_2d(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2);
double distance_2d(const autonomy_ros2_message::msg::StereoPoint &pt);

Eigen::Matrix3d quaternion_to_rotation_matrix(const cv::Mat &quaternion);
Eigen::Vector3d rotation_matrix_to_euler_angle_with_rpy(const Eigen::Matrix3d &rotation);

void pub_cloud_msg(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, const std::string &frame_id, const rclcpp::Time &time);
void pub_custom_msg(const rclcpp::Publisher<autonomy_ros2_message::msg::StereoObject>::SharedPtr &pub, const autonomy_ros2_message::msg::StereoObject::SharedPtr &input, const std::string &frame_id, const rclcpp::Time &time);

pcl::PointXYZI rotate_yaw(const double &yaw, pcl::PointXYZI &pt);
pcl::PointXYZI rotate_pitch(const double &pitch, pcl::PointXYZI &pt);
pcl::PointXYZI rotate_roll(const double &roll, pcl::PointXYZI &pt);
pcl::PointXYZI translation(const double &x_coord, const double &y_coord, const double &z_coord, pcl::PointXYZI &pt);

#endif // __RANGE_UTIL_H__
