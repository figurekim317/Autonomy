#ifndef __SEGMENTATION_VISUALIZER_H__
#define __SEGMENTATION_VISUALIZER_H__

#include <thread>

#include <opencv2/core.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_occupancy_grid.hpp"
#include "test/publisher.h"

constexpr double SEG_WIDTH = 80.0;
constexpr double SEG_HEIGHT = 60.0;

class SegmentationVisualizerNode
{
public:
    SegmentationVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub);

    virtual ~SegmentationVisualizerNode();

    void run_node();

private:
    void declare_ros_parameter();

    void set_segmentation_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &subscriber);

    void segmentation_callback(const autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr input);

    void visualizer_thread();

    rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr m_segmentation_subscriber;

    std::string m_segmentation_topic_to_subscribe;

    bool m_init_segmentation = false;
    Eigen::Matrix3d m_distortion_correction_matrix;

    std::mutex m_segmentation_mutex;

    autonomy_ros2_message::msg::NEUOccupancyGrid m_segmentation_data;

    rclcpp::Node &m_node;
    std::shared_ptr<PublisherNode> m_pub;

    std::thread m_thread;
};

#endif // __SEGMENTATION_VISUALIZER_H__
