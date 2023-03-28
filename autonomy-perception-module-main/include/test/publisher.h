#ifndef __PUBLISHER_H__
#define __PUBLISHER_H__

#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_detection.hpp"
#include "autonomy_ros2_message/msg/neu_occupancy_grid.hpp"

constexpr int STACK_SIZE = 15;
constexpr int CROP = 0;
constexpr double WIDTH = 2592.0;
constexpr double HEIGHT = 1944.0;
constexpr double FPS = 15.0;
constexpr double FPS_FRATION = 1.0 / FPS;

struct MatStamped
{
    cv::Mat matrix;
    rclcpp::Time stamp;
};

class PublisherNode
{
public:
    PublisherNode(rclcpp::Node &node);

    virtual ~PublisherNode();

    void run_node();

    void get_obstacles(MatStamped input);
    void get_detection(MatStamped input);
    void get_segmentation(MatStamped input);
    void get_img(MatStamped input);

private:
    void declare_ros_parameter();

    void set_result_publisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher);

    void synchronize_data(const std::vector<MatStamped> src_vec, const std::vector<MatStamped> dst_vec,
                          MatStamped &src, MatStamped &dst, bool &sync);

    // void show_result(MatStamped src_img, std::string window_name, int widow_size_width, int widow_size_height);

    void visualizer_thread();

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_img_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_msg_publisher;
    std::string m_msg_topic_to_publish;

    std::string m_frame_id_name;

    bool m_init_custom = false;
    bool m_init_detection = false;
    bool m_init_seg = false;
    bool m_init_img = false;
    Eigen::Matrix3d m_distortion_correction_matrix;

    std::mutex m_custom_mutex;
    std::mutex m_detection_mutex;
    std::mutex m_seg_mutex;
    std::mutex m_img_mutex;

    std::vector<MatStamped> m_point_cloud_data_vec;
    std::vector<MatStamped> m_detection_data_vec;
    std::vector<MatStamped> m_seg_data_vec;
    std::vector<MatStamped> m_img_data_vec;

    rclcpp::Node &m_node;

    std::thread m_thread;
};

#endif // __PUBLISHER_H__
