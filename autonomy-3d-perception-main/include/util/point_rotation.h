#ifndef __POINT_ROTATION_H__
#define __POINT_ROTATION_H__

#include "util/range_util.h"
#include "depth_extrinsic_parameter_manager.h"

constexpr int STACK_SIZE = 3;

class PointRotationNode : public rclcpp::Node
{

public:
    PointRotationNode(std::string node_name);

    virtual ~PointRotationNode();

    void run_node();

private:
    void get_ros_parameter();

    void set_point_subscriber(rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr &subscriber);
    void set_point_publisher(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);

    void calculate_euler_angle();

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr &input);

    void point_rotation_thread();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_publisher;

    std::string m_point_topic_to_publish;
    std::string m_point_topic_to_subcribe;
    std::string m_frame_id_name;
    double m_pitch;
    double m_roll;
    bool m_left;
    bool m_init_point;

    std::mutex m_point_mutex;

    sensor_msgs::msg::PointCloud2 m_point_cloud_data;

    std::thread m_thread;
};

#endif // __POINT_ROTATION_H__
