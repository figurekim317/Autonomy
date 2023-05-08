





#ifndef __COMBINE_OBJECT_H__
#define __COMBINE_OBJECT_H__

#include <mutex>

#include "autonomy_ros2_message/msg/depth_alive.hpp"
#include "util/range_util.h"

constexpr double LIMIT_HEIGHT = 1.0;

class CombineObjectNode
{
public:
    CombineObjectNode(rclcpp::Node &node);

    virtual ~CombineObjectNode();

    void run_node();

private:
    void get_ros_parameter();
    void set_left_point_subscriber(rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr &subscriber);
    void set_right_point_subscriber(rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr &subscriber);
    void set_point_publisher(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);
    void set_left_alive_publisher(rclcpp::Publisher<autonomy_ros2_message::msg::DepthAlive>::SharedPtr &publisher);
    void set_right_alive_publisher(rclcpp::Publisher<autonomy_ros2_message::msg::DepthAlive>::SharedPtr &publisher);


    void calculate_euler_angle();

    void convert_to_base_frame(const sensor_msgs::msg::PointCloud2 &input, pcl::PointCloud<pcl::PointXYZI>::Ptr &output,
                               const double &roll, const double &pitch, const double &yaw,
                               const double &x_coord, const double &y_coord, const double &z_coord);

    void combine_left_callback(const sensor_msgs::msg::PointCloud2::SharedPtr &input);
    void combine_right_callback(const sensor_msgs::msg::PointCloud2::SharedPtr &input);

    void combine_object_thread();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_left_point_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_right_point_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_publisher;
    rclcpp::Publisher<autonomy_ros2_message::msg::DepthAlive>::SharedPtr m_left_alive_publisher;
    rclcpp::Publisher<autonomy_ros2_message::msg::DepthAlive>::SharedPtr m_right_alive_publisher;

    double m_frame_left_yaw_deg;
    double m_frame_right_yaw_deg;
    double m_frame_left_pitch_deg;
    double m_frame_right_pitch_deg;
    double m_frame_left_roll_deg;
    double m_frame_right_roll_deg;
    double m_left_x;
    double m_left_y;
    double m_left_z;
    double m_right_x;
    double m_right_y;
    double m_right_z;
    double m_base_x;
    double m_base_y;
    double m_base_z;

    std::string m_point_topic_to_publish;
    std::string m_left_point_topic_to_subcribe;
    std::string m_right_point_topic_to_subcribe;
    std::string m_left_alive_topic_to_publish;
    std::string m_right_alive_topic_to_publish;
    std::string m_frame_id_name;

    bool m_init_left = false;
    bool m_init_right = false;
    int m_number_left = 0;
    int m_number_right = 0;

    std::mutex m_left_mutex;
    std::mutex m_right_mutex;

    rclcpp::Node &m_node;

    sensor_msgs::msg::PointCloud2 m_point_cloud_data_left;
    sensor_msgs::msg::PointCloud2 m_point_cloud_data_right;

    std::thread m_thread;
};

#endif // __COMBINE_OBJECT_H__
