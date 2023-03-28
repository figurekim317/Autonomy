#include "range_util.h"

pcl::PointXYZI input_point(pcl::PointXYZI &point)
{
    pcl::PointXYZI res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = point.intensity;

    return res;
}

pcl::PointXYZI input_point(autonomy_ros2_message::msg::StereoPoint &point)
{
    pcl::PointXYZI res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = point.intensity;

    return res;
}

autonomy_ros2_message::msg::StereoPoint input_custom_point(const autonomy_ros2_message::msg::StereoPoint &point)
{
    autonomy_ros2_message::msg::StereoPoint res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = point.intensity;

    return res;
}

autonomy_ros2_message::msg::StereoPoint input_custom_point(const pcl::PointXYZI &point)
{
    autonomy_ros2_message::msg::StereoPoint res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = point.intensity;

    return res;
}

pcl::PointXYZI input_point_with_intensity(const pcl::PointXYZ &point, const float &intensity)
{
    pcl::PointXYZI res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = intensity;

    return res;
}

pcl::PointXYZI input_point_with_intensity(const pcl::PointXYZI &point, const float &intensity)
{
    pcl::PointXYZI res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = intensity;

    return res;
}

pcl::PointXYZI input_point_with_intensity(const autonomy_ros2_message::msg::StereoPoint &point, const float &intensity)
{
    pcl::PointXYZI res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = intensity;

    return res;
}

autonomy_ros2_message::msg::StereoPoint input_custom_point_with_intensity(const pcl::PointXYZI &point, const float &intensity)
{
    autonomy_ros2_message::msg::StereoPoint res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = intensity;

    return res;
}

autonomy_ros2_message::msg::StereoPoint input_custom_point_with_intensity(const autonomy_ros2_message::msg::StereoPoint &point, const float &intensity)
{
    autonomy_ros2_message::msg::StereoPoint res;

    res.x = point.x;
    res.y = point.y;
    res.z = point.z;
    res.intensity = intensity;

    return res;
}

double distance(const pcl::PointXYZI &pt)
{
    return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

double distance(const autonomy_ros2_message::msg::StereoPoint &pt)
{
    return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

double distance(const autonomy_ros2_message::msg::StereoPoint &before, const autonomy_ros2_message::msg::StereoPoint &after)
{
    return sqrt((after.x - before.x) * (after.x - before.x) + (after.y - before.y) * (after.y - before.y) + (after.z - before.z) * (after.z - before.z));
}

double distance(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
{
    return sqrt((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y) + (pt2.z - pt1.z) * (pt2.z - pt1.z));
}

double distance_2d(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
{
    return sqrt((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y));
}

double distance_2d(const autonomy_ros2_message::msg::StereoPoint &pt)
{
    return sqrt(pt.x * pt.x + pt.y * pt.y);
}

Eigen::Matrix3d quaternion_to_rotation_matrix(const cv::Mat &quaternion)
{
    Eigen::Matrix3d rot_matrix;

    rot_matrix << 1 - 2 * (quaternion.at<double>(0, 1) * quaternion.at<double>(0, 1) + quaternion.at<double>(0, 2) * quaternion.at<double>(0, 2)), 2 * (quaternion.at<double>(0, 0) * quaternion.at<double>(0, 1) - quaternion.at<double>(0, 2) * quaternion.at<double>(0, 3)), 2 * (quaternion.at<double>(0, 0) * quaternion.at<double>(0, 2) + quaternion.at<double>(0, 3) * quaternion.at<double>(0, 1)),
        2 * (quaternion.at<double>(0, 0) * quaternion.at<double>(0, 1) + quaternion.at<double>(0, 2) * quaternion.at<double>(0, 3)), 1 - 2 * (quaternion.at<double>(0, 0) * quaternion.at<double>(0, 0) + quaternion.at<double>(0, 2) * quaternion.at<double>(0, 2)), 2 * (quaternion.at<double>(0, 1) * quaternion.at<double>(0, 2) - quaternion.at<double>(0, 3) * quaternion.at<double>(0, 0)),
        2 * (quaternion.at<double>(0, 0) * quaternion.at<double>(0, 2) - quaternion.at<double>(0, 3) * quaternion.at<double>(0, 1)), 2 * (quaternion.at<double>(0, 1) * quaternion.at<double>(0, 2) + quaternion.at<double>(0, 3) * quaternion.at<double>(0, 0)), 1 - 2 * (quaternion.at<double>(0, 0) * quaternion.at<double>(0, 0) + quaternion.at<double>(0, 1) * quaternion.at<double>(0, 1));

    return rot_matrix;
}

Eigen::Vector3d rotation_matrix_to_euler_angle_with_rpy(const Eigen::Matrix3d &rotation)
{
    Eigen::Vector3d rpy(atan2(-rotation(1, 2), rotation(1, 1)), atan2(-rotation(2, 0), rotation(0, 0)), asin(rotation(1, 0))); // YZX

    return rpy;
}

void pub_cloud_msg(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, const std::string &frame_id, const rclcpp::Time &time)
{
    sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*input, *output);
    output->header.frame_id = frame_id;
    output->header.stamp = time;
    pub->publish(*output);
}

void pub_custom_msg(const rclcpp::Publisher<autonomy_ros2_message::msg::StereoObject>::SharedPtr &pub, const autonomy_ros2_message::msg::StereoObject::SharedPtr &input, const std::string &frame_id, const rclcpp::Time &time)
{
    input->header.frame_id = frame_id;
    input->header.stamp = time;
    pub->publish(*input);
}

pcl::PointXYZI rotate_yaw(const double &yaw, pcl::PointXYZI &pt)
{
    Eigen::Matrix4d transform_yaw = Eigen::Matrix4d::Identity();

    transform_yaw(0, 0) = std::cos(yaw);
    transform_yaw(0, 1) = -sin(yaw);
    transform_yaw(1, 0) = sin(yaw);
    transform_yaw(1, 1) = std::cos(yaw);

    Eigen::Vector4d in(pt.x, pt.y, pt.z, 1);
    Eigen::Vector4d out = transform_yaw * in;

    pcl::PointXYZI res;

    res.x = out[0];
    res.y = out[1];
    res.z = out[2];
    res.intensity = pt.intensity;

    return res;
}

pcl::PointXYZI rotate_pitch(const double &pitch, pcl::PointXYZI &pt)
{

    Eigen::Matrix4d transform_pitch = Eigen::Matrix4d::Identity();

    transform_pitch(0, 0) = std::cos(pitch);
    transform_pitch(0, 2) = sin(pitch);
    transform_pitch(2, 0) = -sin(pitch);
    transform_pitch(2, 2) = std::cos(pitch);

    Eigen::Vector4d in(pt.x, pt.y, pt.z, 1);
    Eigen::Vector4d out = transform_pitch * in;

    pcl::PointXYZI res;

    res.x = out[0];
    res.y = out[1];
    res.z = out[2];
    res.intensity = pt.intensity;

    return res;
}

pcl::PointXYZI rotate_roll(const double &roll, pcl::PointXYZI &pt)
{

    Eigen::Matrix4d transform_roll = Eigen::Matrix4d::Identity();

    transform_roll(1, 1) = std::cos(roll);
    transform_roll(1, 2) = -sin(roll);
    transform_roll(2, 1) = sin(roll);
    transform_roll(2, 2) = std::cos(roll);

    Eigen::Vector4d in(pt.x, pt.y, pt.z, 1);
    Eigen::Vector4d out = transform_roll * in;

    pcl::PointXYZI res;

    res.x = out[0];
    res.y = out[1];
    res.z = out[2];
    res.intensity = pt.intensity;

    return res;
}

pcl::PointXYZI translation(const double &x_coord, const double &y_coord, const double &z_coord, pcl::PointXYZI &pt)
{
    Eigen::Matrix4d transform_translation = Eigen::Matrix4d::Identity();

    transform_translation(0, 3) = x_coord;
    transform_translation(1, 3) = y_coord;
    transform_translation(2, 3) = z_coord;

    Eigen::Vector4d in(pt.x, pt.y, pt.z, 1);
    Eigen::Vector4d out = transform_translation * in;

    pcl::PointXYZI res;

    res.x = out[0];
    res.y = out[1];
    res.z = out[2];
    res.intensity = pt.intensity;

    return res;
}
