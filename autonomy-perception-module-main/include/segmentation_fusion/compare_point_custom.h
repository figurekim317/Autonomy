#ifndef __COMPARE_POINT_H__
#define __COMPARE_POINT_H__

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "range_util.h"
#include "autonomy_ros2_message/msg/neu_occupancy_grid.hpp"

constexpr int STACK_SIZE = 5;
constexpr int WIDTH = 160;
constexpr int HEIGHT = 120;
constexpr double FPS = 15;
constexpr double FPS_FRATION = 1 / FPS;
constexpr int HEIGHT_ASPECT_RATE = 30;

class ComparePointNode
{

public:
    ComparePointNode(rclcpp::Node &node);

    virtual ~ComparePointNode();

    void run_node();

    Eigen::Matrix3d get_camera_matrix() const { return m_distortion_correction_matrix; };

private:
    void get_ros_parameter();

    void set_ground_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &subscriber);
    void set_neu_grid_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &subscriber);
    void set_result_publisher(rclcpp::Publisher<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &publisher, std::string topic_name);

    void set_point_publisher(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher, std::string topic_name);
    void set_nav_grid_publisher(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &publisher, std::string topic_name);

    void set_camera_parameter(const std::string cam_name);

    void ground_callback(const autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr &input);
    void segmentation_callback(const autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr &input);

    void synchronize_data(const std::vector<autonomy_ros2_message::msg::NEUOccupancyGrid> grid_vec, const std::vector<autonomy_ros2_message::msg::NEUOccupancyGrid> ground_vec,
                          autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr &custom_grid, autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr &custom_ground);

    void project_to_image(pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_input, const autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr seg_input, autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr &seg_output);

    void update_grid(nav_msgs::msg::OccupancyGrid::SharedPtr grid, std::vector<int8_t> occupancy_data, rclcpp::Time stamp);

    void seg_fusion_thread();

    rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr m_ground_subscriber;
    rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr m_segmentation_subscriber;
    rclcpp::Publisher<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr m_neu_grid_publisher;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_nav_publisher;

    std::string m_neu_grid_topic_to_publish;
    std::string m_ground_topic_to_subscribe;
    std::string m_segmentation_topic_to_subscribe;
    std::string m_frame_id_name;
    bool m_init_ground = false;
    bool m_init_seg = false;

    bool m_visualize;
    std::string m_erosion_point_topic_to_publish;
    std::string m_nav_grid_topic_to_publish;

    std::mutex m_ground_mutex;
    std::mutex m_seg_mutex;

    rclcpp::Node &m_node;

    double m_base_x;
    double m_base_y;
    double m_base_z;

    std::vector<autonomy_ros2_message::msg::NEUOccupancyGrid> m_ground_data_vec;
    std::vector<autonomy_ros2_message::msg::NEUOccupancyGrid> m_seg_data_vec;

    Eigen::Matrix3d m_distortion_correction_matrix;

    std::thread m_thread;
};

#endif // __COMPARE_POINT_H__
