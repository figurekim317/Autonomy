#include "image_dimension_transfomration/transformer.h"
#include "camera_parameter_manager.h"
#include "perception2d_class_manager.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>

Transformer::Transformer(std::vector<std::string> cam_names)
    : m_cam_names(cam_names)
{
    for (std::string &cam_name : m_cam_names)
    {
        m_intrinsic_matrix_map[cam_name] = CameraParameterReceiver::instance().get_cam_map()[cam_name].get_intrinsic_matrix();
        std::cout << "cam_" << cam_name << ' ' << "intrinsic_matrix_map" << std::endl;
        std::cout << m_intrinsic_matrix_map[cam_name] << std::endl;
        m_robot_to_cam_rotation_matrix_map[cam_name] = CameraParameterReceiver::instance().get_transform_map()[cam_name].get_robot_to_cam_rotation_matrix();
        std::cout << "cam_" << cam_name << ' ' << "robot_to_cam_rotation_matrix_map" << std::endl;
        std::cout << m_robot_to_cam_rotation_matrix_map[cam_name] << std::endl;
        m_robot_to_cam_translation_vector_map[cam_name] = CameraParameterReceiver::instance().get_transform_map()[cam_name].get_robot_to_cam_translation_vector();
        std::cout << "cam_" << cam_name << ' ' << "robot_to_cam_translation_vector_map" << std::endl;
        std::cout << m_robot_to_cam_translation_vector_map[cam_name] << std::endl;
        m_cam_f_to_cam_rotation_matrix_map[cam_name] = CameraParameterReceiver::instance().get_transform_map()[cam_name].get_cam_f_to_cam_rotation_matrix();
        std::cout << "cam_" << cam_name << ' ' << "cam_f_to_cam_rotation_matrix_map" << std::endl;
        std::cout << m_cam_f_to_cam_rotation_matrix_map[cam_name] << std::endl;
        m_cam_f_to_cam_translation_vector_map[cam_name] = CameraParameterReceiver::instance().get_transform_map()[cam_name].get_cam_f_to_cam_transltation_vector();
        std::cout << "cam_" << cam_name << ' ' << "cam_f_to_cam_translation_vector_map" << std::endl;
        std::cout << m_cam_f_to_cam_translation_vector_map[cam_name] << std::endl;
    }
}

Transformer::~Transformer() {}

double Transformer::get_current_scale_factor()
{
    return this->m_current_scale_factor;
}

bool Transformer::is_over_principal_point(const std::string &cam_name, const cv::Point2d &image_point)
{
    const double &cy = m_intrinsic_matrix_map[cam_name].at<double>(1, 2);
    return image_point.y <= cy;
}

cv::Point3d Transformer::transform_cam_to_robot_using_3d_transformation(std::string &cam_name, cv::Point2d &point_2d, double z_axis_3d)
{
    /*
        reference url : https://stackoverflow.com/questions/14514357/converting-a-2d-image-point-to-a-3d-world-point
    */

    const double fx = m_intrinsic_matrix_map[cam_name].at<double>(0, 0);
    const double fy = m_intrinsic_matrix_map[cam_name].at<double>(1, 1);
    const double cx = m_intrinsic_matrix_map[cam_name].at<double>(0, 2);
    const double cy = m_intrinsic_matrix_map[cam_name].at<double>(1, 2);

    cv::Mat rotation_matrix = m_robot_to_cam_rotation_matrix_map["f"].inv() * m_cam_f_to_cam_rotation_matrix_map[cam_name];
    cv::Mat point_in_normalized_plane = (cv::Mat_<double>(3, 1) << (point_2d.x - cx) / fx, (point_2d.y - cy) / fy, 1);
    cv::Mat D_xyz = rotation_matrix * point_in_normalized_plane;

    const double dx = D_xyz.at<double>(0, 0);
    const double dy = D_xyz.at<double>(1, 0);
    const double dz = D_xyz.at<double>(2, 0);

    const double Tx = m_robot_to_cam_translation_vector_map["f"].at<double>(0, 0) + m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(2, 0);
    const double Ty = m_robot_to_cam_translation_vector_map["f"].at<double>(1, 0) + (-m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(0, 0));
    const double Tz = m_robot_to_cam_translation_vector_map["f"].at<double>(2, 0) + (-m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(1, 0));

    const double scale_factor = (z_axis_3d - Tz) / dz;
    this->m_current_scale_factor = scale_factor;

    cv::Point3d point_3d = cv::Point3d(scale_factor * dx + Tx, scale_factor * dy + Ty, z_axis_3d);

    return point_3d;
}

cv::Point2d Transformer::transform_robot_to_cam_using_3d_transformation(std::string &cam_name, cv::Point3d &point_3d)
{
    cv::Mat rotation_matrix = m_cam_f_to_cam_rotation_matrix_map[cam_name].inv() * m_robot_to_cam_rotation_matrix_map["f"];

    const double Tx = m_robot_to_cam_translation_vector_map["f"].at<double>(0, 0) + m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(2, 0);
    const double Ty = m_robot_to_cam_translation_vector_map["f"].at<double>(1, 0) + (-m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(0, 0));
    const double Tz = m_robot_to_cam_translation_vector_map["f"].at<double>(2, 0) + (-m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(1, 0));

    const double dx_dz = (point_3d.x - Tx) / (point_3d.z - Tz);
    const double dy_dz = (point_3d.y - Ty) / (point_3d.z - Tz);
    const double dz = 1. / ((rotation_matrix.at<double>(2, 0) * dx_dz + rotation_matrix.at<double>(2, 1) * dy_dz + rotation_matrix.at<double>(2, 2)));
    const double dx = dx_dz * dz;
    const double dy = dy_dz * dz;

    cv::Mat D_xyz = (cv::Mat_<double>(3, 1) << dx, dy, dz);
    cv::Mat _point_2d_in_normalized_plane = rotation_matrix * D_xyz;
    cv::Point2d point_2d_in_normalized_plane = cv::Point2d(_point_2d_in_normalized_plane.at<double>(0, 0), _point_2d_in_normalized_plane.at<double>(1, 0));

    const double fx = m_intrinsic_matrix_map[cam_name].at<double>(0, 0);
    const double fy = m_intrinsic_matrix_map[cam_name].at<double>(1, 1);
    const double cx = m_intrinsic_matrix_map[cam_name].at<double>(0, 2);
    const double cy = m_intrinsic_matrix_map[cam_name].at<double>(1, 2);

    cv::Point2d point_2d = cv::Point(point_2d_in_normalized_plane.x * fx + cx, point_2d_in_normalized_plane.y * fy + cy);

    return point_2d;
}

cv::Point2d Transformer::transform_robot_to_cam_using_3d_transformation(std::string &cam_name, cv::Point3d &point_3d, double scale_factor)
{
    cv::Mat rotation_matrix = m_cam_f_to_cam_rotation_matrix_map[cam_name].inv() * m_robot_to_cam_rotation_matrix_map["f"];

    const double Tx = m_robot_to_cam_translation_vector_map["f"].at<double>(0, 0) + m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(2, 0);
    const double Ty = m_robot_to_cam_translation_vector_map["f"].at<double>(1, 0) + (-m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(0, 0));
    const double Tz = m_robot_to_cam_translation_vector_map["f"].at<double>(2, 0) + (-m_cam_f_to_cam_translation_vector_map[cam_name].at<double>(1, 0));

    const double dx = (point_3d.x - Tx) / scale_factor;
    const double dy = (point_3d.y - Ty) / scale_factor;
    const double dz = (point_3d.z - Tz) / scale_factor;

    cv::Mat D_xyz = (cv::Mat_<double>(3, 1) << dx, dy, dz);
    cv::Mat _point_2d_in_normalized_plane = rotation_matrix * D_xyz;
    cv::Point2d point_2d_in_normalized_plane = cv::Point2d(_point_2d_in_normalized_plane.at<double>(0, 0), _point_2d_in_normalized_plane.at<double>(1, 0));

    const double fx = m_intrinsic_matrix_map[cam_name].at<double>(0, 0);
    const double fy = m_intrinsic_matrix_map[cam_name].at<double>(1, 1);
    const double cx = m_intrinsic_matrix_map[cam_name].at<double>(0, 2);
    const double cy = m_intrinsic_matrix_map[cam_name].at<double>(1, 2);

    cv::Point2d point_2d = cv::Point(point_2d_in_normalized_plane.x * fx + cx, point_2d_in_normalized_plane.y * fy + cy);

    return point_2d;
}

cv::Point3d Transformer::transform_cam_to_robot_using_prior_height(std::string &cam_name, cv::Point2d &bcenter, cv::Point2d &bsize, double prior_height)
{
    const double fx = m_intrinsic_matrix_map[cam_name].at<double>(0, 0);
    const double fy = m_intrinsic_matrix_map[cam_name].at<double>(1, 1);
    const double cx = m_intrinsic_matrix_map[cam_name].at<double>(0, 2);
    const double bbox_height = bsize.y;

    cv::Point2d point_2d;

    const double z_c = fy * (prior_height / bbox_height);
    point_2d.x = z_c;
    point_2d.y = z_c * (cx - bcenter.x) / fx;

    cv::Mat prior_point = (cv::Mat_<double>(3, 1) << -point_2d.y, 0.0, point_2d.x);

    cv::Mat D_xyz = m_robot_to_cam_rotation_matrix_map["f"].inv() * m_cam_f_to_cam_rotation_matrix_map[cam_name] * prior_point;
    cv::Point3d point_3d = cv::Point3d(D_xyz.at<double>(0, 0), D_xyz.at<double>(1, 0), 0);

    return point_3d;
}
