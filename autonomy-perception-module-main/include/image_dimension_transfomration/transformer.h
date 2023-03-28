#ifndef __TRANSFORMER_H__
#define __TRANSFORMER_H__

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <opencv2/core.hpp>

class Transformer
{
public:
    Transformer(std::vector<std::string> cam_names);
    ~Transformer();

    cv::Point3d transform_cam_to_robot_using_3d_transformation(std::string &cam_name, cv::Point2d &point_2d, double z_axis_3d);
    cv::Point2d transform_robot_to_cam_using_3d_transformation(std::string &cam_name, cv::Point3d &point_3d);
    cv::Point2d transform_robot_to_cam_using_3d_transformation(std::string &cam_name, cv::Point3d &point_3d, double scale_factor);
    cv::Point3d transform_cam_to_robot_using_prior_height(std::string &cam_name, cv::Point2d &point_2d, cv::Point2d &bsize, double prior_height);

    double get_current_scale_factor();
    bool is_over_principal_point(const std::string &cam_name, const cv::Point2d &image_point);

private:
    std::vector<std::string> m_cam_names;

    std::map<std::string, cv::Mat> m_intrinsic_matrix_map;
    std::map<std::string, cv::Mat> m_robot_to_cam_rotation_matrix_map;
    std::map<std::string, cv::Mat> m_robot_to_cam_translation_vector_map;
    std::map<std::string, cv::Mat> m_cam_f_to_cam_rotation_matrix_map;
    std::map<std::string, cv::Mat> m_cam_f_to_cam_translation_vector_map;

    double m_current_scale_factor;
};

#endif // __TRANSFORMER_H__
