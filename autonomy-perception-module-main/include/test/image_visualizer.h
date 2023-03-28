#ifndef __IMAGE_VISUALIZER_H__
#define __IMAGE_VISUALIZER_H__

#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_detection.hpp"
#include "test/publisher.h"

class ImageVisualizerNode
{
public:
    ImageVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub);

    virtual ~ImageVisualizerNode();

    void run_node();

private:
    void declare_ros_parameter();

    void set_img_subscriber(rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr &subscriber);
    void img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr &input);
    void convert_compressed_to_cv(const sensor_msgs::msg::CompressedImage::SharedPtr src, cv::Mat &dst);

    void visualizer_thread();

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_img_subscriber;

    std::string m_img_topic_to_subscribe;

    bool m_init_img = false;

    std::mutex m_img_mutex;

    sensor_msgs::msg::CompressedImage m_img_data;

    rclcpp::Node &m_node;
    std::shared_ptr<PublisherNode> m_pub;

    std::thread m_thread;
};

#endif // __IMAGE_VISUALIZER_H__
