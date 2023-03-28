#ifndef __INVERSE_PERSPECTIVE_MAPPING_H__
#define __INVERSE_PERSPECTIVE_MAPPING_H__

#include <opencv2/opencv.hpp>
#include <string>
#include "camera_info/neu_camera_info.h"
class InversePerspectiveMapping
{
public:
    InversePerspectiveMapping();
    InversePerspectiveMapping(const std::string &cam_name);
    void undistort(int *image, const unsigned int width, const unsigned int height, const std::vector<unsigned short> &crop_params, std::vector<signed char> &res);
    void transform(int *image, const int width, const int height, const std::vector<unsigned short> &crop_params, std::vector<unsigned char> &res);

private:
    int m_grid_width = 20;
    int m_grid_height = 20;
    double m_grid_size = 0.1;
    int m_image_width = NeuCameraInfo::get_instance().get_camera_width("cam_f");
    int m_image_height = NeuCameraInfo::get_instance().get_camera_height("cam_f");

    cv::Size m_image_size = cv::Size(m_image_width, m_image_height);

    cv::Mat m_new_camera_matrix, m_projection_matrix, m_map_x, m_map_y, m_image_points, m_grid_xy;
    cv::Mat m_camera_matrix, m_distortion_coefficients;
    cv::Mat m_imu_rotation = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat m_imu_translation = cv::Mat(3, 1, CV_32FC1);

    std::vector<bool> m_points_in_image_index;

    void set_calibration();
    void set_optimal_new_camera_matrix();
    void calc_projection_matrix();
    void calc_undistort_remap();
    void calc_image_points();
    void aspect_reshape(int *image, const int width, const int height, const std::vector<unsigned short> &crop_params, cv::Mat &padding_image) const;
    void get_perspective_image_u(std::vector<int> &result);
    void get_perspective_image_v(std::vector<int> &result);
    void get_linspace(const double &start, const double &end, const int &count, std::vector<double> &result) const;
    void get_world_points(cv::Mat &result) const;
    void is_points_in_image(std::vector<bool> &vec);

    const cv::Mat initialize_image_matrix() const;
    const cv::Mat get_camera_matrix() const;
    const cv::Mat get_distortion_coefficientes() const;
    const cv::Mat get_optimal_new_camera_matrix() const;
    const cv::Mat get_projection_matrix() const;
    const cv::Mat get_remap_x() const;
    const cv::Mat get_remap_y() const;

    const std::vector<bool> get_points_in_image_index() const;
};

#endif //__INVERSE_PERSPECTIVE_MAPPING_H__
