#ifndef __OBSTACLES_VISUALIZER_H__
#define __OBSTACLES_VISUALIZER_H__

#include <thread>

#include <opencv2/core.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_detection.hpp"
#include "test/publisher.h"

class ObstaclesVisualizerNode
{
public:
    ObstaclesVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub);

    virtual ~ObstaclesVisualizerNode();

    void run_node();

    Eigen::Matrix3d get_camera_matrix() const;

private:
    void declare_ros_parameter();

    void set_custom_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr &subscriber);

    void set_camera_parameter(const std::string cam_name);

    void obstacles_callback(const autonomy_ros2_message::msg::StereoObject::SharedPtr input);

    void draw_bbox_rectangle(const double max_x, const double max_y, const double min_x, const double min_y, cv::Mat &src_img);

    void project_to_image(const autonomy_ros2_message::msg::StereoObject::SharedPtr obs, cv::Mat &pc_img);

    void visualizer_thread();

    rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr m_custom_subscriber;

    std::string m_custom_topic_to_subscribe;

    bool m_init_custom = false;
    Eigen::Matrix3d m_distortion_correction_matrix;

    std::mutex m_custom_mutex;

    autonomy_ros2_message::msg::StereoObject m_point_cloud_data;

    rclcpp::Node &m_node;
    std::shared_ptr<PublisherNode> m_pub;

    std::thread m_thread;
};

#endif // __OBSTACLES_VISUALIZER_H__
