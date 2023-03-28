#include "camera_calibration/inverse_perspective_mapping.h"
#include "camera_parameter_manager.h"
#include <cmath>

InversePerspectiveMapping::InversePerspectiveMapping()
{
}

InversePerspectiveMapping::InversePerspectiveMapping(const std::string &cam_name) : m_image_width(NeuCameraInfo::get_instance().get_camera_width(cam_name)), m_image_height(NeuCameraInfo::get_instance().get_camera_height(cam_name))
{
    float *ptr = &m_imu_translation.at<float>(0, 0);
    ptr[0] = 0.23f;
    ptr[1] = 0;
    ptr[2] = 0.41f;

    set_calibration();
    set_optimal_new_camera_matrix();
    calc_projection_matrix();
    calc_undistort_remap();
    calc_image_points();

    cv::Mat world_points;
    get_world_points(world_points);
    m_grid_xy = cv::Mat(cv::Size(2, world_points.cols), CV_32SC1);
    int *grid_ptr = &m_grid_xy.at<int>(0, 0);
    float *world_points_ptr = &world_points.at<float>(0, 0);
    int idx = 0;

    for (int i = 0; i < world_points.cols; i++)
    {
        grid_ptr[idx] = std::round((world_points_ptr[2 * world_points.cols + i] - m_grid_size / 2. + m_grid_height / 2.) / m_grid_size);
        grid_ptr[idx + 1] = std::round((world_points_ptr[i] - m_grid_size / 2. + m_grid_width / 2.) / m_grid_size);
        idx += 2;
    }

    is_points_in_image(m_points_in_image_index);
}

void InversePerspectiveMapping::set_calibration()
{
    CameraParameterReceiver camera_parameter_receiver = CameraParameterReceiver::instance();
    m_camera_matrix = camera_parameter_receiver.get_cam_map()["f"].get_intrinsic_matrix();
    m_distortion_coefficients = camera_parameter_receiver.get_cam_map()["f"].get_distortion_matrix();
}

void InversePerspectiveMapping::set_optimal_new_camera_matrix()
{
    m_new_camera_matrix = cv::getOptimalNewCameraMatrix(m_camera_matrix, m_distortion_coefficients, m_image_size, 0.0, m_image_size, 0);
}

void InversePerspectiveMapping::calc_projection_matrix()
{
    cv::Mat m_imu;
    cv::hconcat(m_imu_rotation, m_imu_translation, m_imu);
    m_new_camera_matrix.convertTo(m_new_camera_matrix, CV_32F);
    m_projection_matrix = m_new_camera_matrix * m_imu;
}

void InversePerspectiveMapping::calc_undistort_remap()
{
    cv::initUndistortRectifyMap(m_camera_matrix, m_distortion_coefficients, cv::Mat(), m_new_camera_matrix, m_image_size, 5, m_map_x, m_map_y);
}

void InversePerspectiveMapping::calc_image_points()
{
    cv::Mat world_points;
    get_world_points(world_points);
    cv::Mat trans;
    cv::transpose(m_projection_matrix * world_points, trans);
    m_image_points = cv::Mat::zeros(cv::Size(2, trans.rows), CV_32SC1);

    int *img_ptr = &m_image_points.at<int>(0, 0);
    float *trans_ptr = &trans.at<float>(0, 0);
    float divide = 0;
    int src_idx = 0;
    int dst_idx = 0;

    for (int i = 0; i < trans.rows; i++)
    {
        divide = trans_ptr[src_idx + 2];
        img_ptr[dst_idx] = (int)(trans_ptr[src_idx + 1] / divide);
        img_ptr[dst_idx + 1] = (int)(trans_ptr[src_idx] / divide);
        src_idx += trans.cols;
        dst_idx += 2;
    }
}

void InversePerspectiveMapping::aspect_reshape(int *image, const int width, const int height, const std::vector<unsigned short> &crop_params, cv::Mat &padding_image) const
{
    int resized_width = m_image_width - crop_params[0];
    int resized_height = m_image_height - crop_params[1];
    cv::Mat img = cv::Mat(cv::Size(width, height), CV_32SC1, image);
    cv::Mat resize_image;
    cv::resize(img, resize_image, cv::Size(resized_width, resized_height), 0, 0, cv::INTER_NEAREST);
    resize_image.convertTo(resize_image, CV_8UC1);
    cv::copyMakeBorder(resize_image, padding_image, crop_params[1], m_image_height - crop_params[1] - resized_height, crop_params[0], m_image_width - crop_params[0] - resized_width, cv::BORDER_CONSTANT);
}

void InversePerspectiveMapping::undistort(int *image, const unsigned int width, const unsigned int height, const std::vector<unsigned short> &crop_params, std::vector<signed char> &res)
{
    cv::Mat reshape_image, undistorted_image, result;
    aspect_reshape(image, width, height, crop_params, reshape_image);
    cv::remap(reshape_image, undistorted_image, m_map_x, m_map_y, cv::INTER_NEAREST);

    cv::Rect bounds(0, 0, undistorted_image.cols, undistorted_image.rows);
    cv::Rect r(crop_params[0], crop_params[1], crop_params[2], crop_params[3]);
    cv::Mat cropped_image = undistorted_image(r & bounds);

    cv::resize(cropped_image, result, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);

    res.assign((signed char *)result.datastart, (signed char *)result.dataend);
}

void InversePerspectiveMapping::transform(int *image, const int width, const int height, const std::vector<unsigned short> &crop_params, std::vector<unsigned char> &res)
{
    cv::Mat reshape_image, undistorted_image;
    aspect_reshape(image, width, height, crop_params, reshape_image);
    cv::remap(reshape_image, undistorted_image, m_map_x, m_map_y, cv::INTER_NEAREST);

    cv::Mat perspective_matrix = initialize_image_matrix();

    std::vector<int> piu, piv;
    get_perspective_image_u(piu);
    get_perspective_image_v(piv);

    int size = m_points_in_image_index.size();
    int idx = 0, x = 0, y = 0, mip_idx = 0;
    int *ptr = &m_image_points.at<int>(0, 0);

    for (int i = 0; i < size; i++)
    {
        if (m_points_in_image_index[i])
        {
            mip_idx = m_image_points.cols * i;
            y = ptr[mip_idx];
            x = ptr[mip_idx + 1];
            perspective_matrix.data[piu[idx] * perspective_matrix.cols + piv[idx]] = undistorted_image.data[y * undistorted_image.cols + x];
            idx++;
        }
    }

    res.assign(perspective_matrix.datastart, perspective_matrix.dataend);
}

const cv::Mat InversePerspectiveMapping::initialize_image_matrix() const
{
    return cv::Mat::zeros(m_grid_height / m_grid_size, m_grid_width / m_grid_size, CV_8SC1);
}

const std::vector<bool> InversePerspectiveMapping::get_points_in_image_index() const
{
    return m_points_in_image_index;
}

void InversePerspectiveMapping::get_perspective_image_u(std::vector<int> &result)
{
    int *xy_ptr = &m_grid_xy.at<int>(0, 0);
    int src_idx = 0;
    int rows = m_points_in_image_index.size();
    result.reserve(rows);

    for (int i = 0; i < rows; i++)
    {
        if (m_points_in_image_index[i])
            result.push_back(xy_ptr[src_idx]);
        src_idx += m_grid_xy.cols;
    }
}

void InversePerspectiveMapping::get_perspective_image_v(std::vector<int> &result)
{
    int *xy_ptr = &m_grid_xy.at<int>(0, 0);
    int src_idx = 1;
    int rows = m_points_in_image_index.size();
    result.reserve(rows);

    for (int i = 0; i < rows; i++)
    {
        if (m_points_in_image_index[i])
            result.push_back(xy_ptr[src_idx]);
        src_idx += m_grid_xy.cols;
    }
}

const cv::Mat InversePerspectiveMapping::get_camera_matrix() const
{
    return m_camera_matrix;
}

const cv::Mat InversePerspectiveMapping::get_distortion_coefficientes() const
{
    return m_distortion_coefficients;
}

const cv::Mat InversePerspectiveMapping::get_optimal_new_camera_matrix() const
{
    return m_new_camera_matrix;
}

const cv::Mat InversePerspectiveMapping::get_projection_matrix() const
{
    return m_projection_matrix;
}

void InversePerspectiveMapping::get_linspace(const double &start, const double &end, const int &count, std::vector<double> &result) const
{
    double step = (double)(end - start) / (count - 1);
    double st = start;

    while (st <= end)
    {
        result.push_back(st);
        st += step;
    }
}

const cv::Mat InversePerspectiveMapping::get_remap_x() const
{
    return m_map_x;
}

const cv::Mat InversePerspectiveMapping::get_remap_y() const
{
    return m_map_y;
}

void InversePerspectiveMapping::get_world_points(cv::Mat &result) const
{
    double start = m_grid_size / 2;
    double end = m_grid_height / 2 - m_grid_size / 2;
    int count = m_grid_height / (2 * m_grid_size);

    std::vector<double> grid_x, grid_y;
    get_linspace(start, end, count, grid_x);

    start = -1 * m_grid_width / 2 + m_grid_size / 2;
    end = m_grid_width / 2 - m_grid_size / 2;
    count = (int)(m_grid_width / m_grid_size);

    get_linspace(start, end, count, grid_y);

    int idx = 0, dst_idx = 0;
    int len_x = grid_x.size();
    int len_y = grid_y.size();
    int total_len = len_x * len_y;

    cv::Mat ux = cv::Mat(1, total_len, CV_32FC1);
    cv::Mat uy = cv::Mat(1, total_len, CV_32FC1);

    float *ux_ptr = &ux.at<float>(0, 0);
    float *uy_ptr = &uy.at<float>(0, 0);

    for (int i = 0; i < len_y; i++)
    {
        for (int j = 0; j < len_x; j++)
        {
            idx = dst_idx + j;
            ux_ptr[idx] = grid_x[j];
            uy_ptr[idx] = grid_y[i];
        }
        dst_idx += len_x;
    }

    cv::Mat uz(cv::Size(ux.cols, ux.rows), CV_32FC1, cv::Scalar(-0.63));
    cv::Mat uv = cv::Mat::ones(cv::Size(ux.cols, ux.rows), CV_32FC1);

    cv::vconcat(-1 * uy, -1 * uz, result);
    cv::vconcat(result, ux, result);
    cv::vconcat(result, uv, result);
}

void InversePerspectiveMapping::is_points_in_image(std::vector<bool> &vec)
{
    int idx = 0;
    int idx2 = 1;
    int *ptr = &m_image_points.at<int>(0, 0);
    vec.reserve(m_image_points.rows);

    for (int i = 0; i < m_image_points.rows; i++)
    {
        vec.push_back(
            (ptr[idx] < m_image_height) &
            (ptr[idx] >= 0) &
            (ptr[idx2] < m_image_width) &
            (ptr[idx2] >= 0));
        idx += m_image_points.cols;
        idx2 += m_image_points.cols;
    }
}
