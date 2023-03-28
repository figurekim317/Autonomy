#ifndef __PANIC_MODE_H__
#define __PANIC_MODE_H__

#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/core/eigen.hpp>
#include <std_msgs/msg/int64.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_occupancy_grid.hpp"


class PanicModeDetectorNode : public rclcpp::Node
{
public:
    PanicModeDetectorNode(const std::string node_name);

    virtual ~PanicModeDetectorNode();

    void run_node();

private:
    void declare_ros_parameter();

    void set_seg_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &subscriber);
    void set_result_publisher(rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr &publisher);

    void seg_callback(const autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr input);

    void pub_status_msg(const int drivable_area_count);

    void detector_thread();

    rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr m_seg_subscriber;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr m_msg_publisher;

    std::string m_seg_topic_to_subscribe;
    std::string m_msg_topic_to_publish;

    bool m_init_segmentation = false;

    std::mutex m_segmentation_mutex;

    autonomy_ros2_message::msg::NEUOccupancyGrid m_segmentation_data;

    std::thread m_thread;
};

#endif // __PANIC_MODE_H__
