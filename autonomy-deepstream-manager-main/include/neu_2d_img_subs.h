#ifndef __NEU_2D_IMG_SUBS_H__
#define __NEU_2D_IMG_SUBS_H__

#include <gst/app/gstappsrc.h>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "neu_deepstream_core.h"

class SubsriberImage
{
public:
    SubsriberImage(const std::string &cam_name);
    ~SubsriberImage();
    GstElement *get_appsrc() const;
    void set_appsrc(GstElement *appsrc);
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr &msg);
    void set_subscriber(rclcpp::Node &node);
    void push_buffer();
    void set_start_subs(const bool use);

private:
    GstElement *m_appsrc;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_subscriber;
    bool m_use;

    std::vector<std::thread> m_thread_pool;
    std::queue<sensor_msgs::msg::CompressedImage> m_img_que;
    std::string m_cam_name;
    std::shared_ptr<NEUPerception> m_perception;
};

#endif //__NEU_2D_IMG_SUBS_H__
