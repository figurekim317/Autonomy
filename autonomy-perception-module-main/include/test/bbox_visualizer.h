#ifndef __BBOX_VISUALIZER_H__
#define __BBOX_VISUALIZER_H__

#include <thread>

#include <opencv2/core.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_detection.hpp"
#include "test/publisher.h"

class BboxVisualizerNode
{
public:
    BboxVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub);

    virtual ~BboxVisualizerNode();

    void run_node();

private:
    void declare_ros_parameter();

    void set_detection_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUDetection>::SharedPtr &subscriber);

    void detection_callback(const autonomy_ros2_message::msg::NEUDetection::SharedPtr &input);

    void draw_bbox_rectangle(const autonomy_ros2_message::msg::NEUBbox neu_box, cv::Mat &src_img);
    void draw_bbox_text(const autonomy_ros2_message::msg::NEUBbox neu_box, cv::Mat &src_img);

    void visualizer_thread();

    rclcpp::Subscription<autonomy_ros2_message::msg::NEUDetection>::SharedPtr m_detection_subscriber;

    std::string m_detection_topic_to_subscribe;

    bool m_init_detection = false;

    std::mutex m_detection_mutex;

    rclcpp::Node &m_node;
    std::shared_ptr<PublisherNode> m_pub;

    autonomy_ros2_message::msg::NEUDetection m_detection_data;

    std::thread m_thread;
};

#endif // __BBOX_VISUALIZER_H__
