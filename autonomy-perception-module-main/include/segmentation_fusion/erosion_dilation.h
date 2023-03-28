#ifndef __EROSION_DILATION_H__
#define __EROSION_DILATION_H__

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_occupancy_grid.hpp"

class ErosionDilationNode
{
public:
    ErosionDilationNode(rclcpp::Node &node);

    virtual ~ErosionDilationNode();

    void run_node();

private:
    void get_ros_parameter();

    void set_point_subscriber(rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr &subscriber);
    void set_neu_grid_publisher(rclcpp::Publisher<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &publisher, std::string topic_name);

    void update_neu_grid(autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr grid, double cell_res, int x_cells, int y_cells,
                         double origin_x, double origin_y, std::vector<int8_t> *occupancy_data, rclcpp::Time stamp);

    void erosion_dilation_callback(const sensor_msgs::msg::PointCloud2::SharedPtr &input);

    void erosion_dilation_thread();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_subscriber;
    rclcpp::Publisher<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr m_neu_grid_publisher;

    std::string m_neu_grid_topic_to_publish;
    std::string m_point_topic_to_subcribe;
    std::string m_frame_id_name;

    bool m_init_point;

    std::mutex m_point_mutex;

    rclcpp::Node &m_node;

    sensor_msgs::msg::PointCloud2 m_point_cloud_data;

    std::thread m_thread;
};

#endif // __EROSION_DILATION_H__
