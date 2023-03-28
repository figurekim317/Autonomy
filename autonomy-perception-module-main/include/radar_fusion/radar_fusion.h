#ifndef __RADAR_FUSION_H__
#define __RADAR_FUSION_H__

#include <thread>

#include <visualization_msgs/msg/marker_array.hpp>

#include "autonomy_ros2_message/msg/radar_scan_obj.hpp"
#include "range_util.h"

constexpr int STACK_SIZE = 3;
constexpr double FPS = 15.0;
constexpr double FPS_FRATION = 1.0 / FPS;
constexpr double ROBOT_HEIGHT = -0.575;
constexpr double BBOX_THRESHOLD = 0.3;

class RadarFusionNode : public rclcpp::Node
{
public:
    RadarFusionNode(std::string node_name);

    virtual ~RadarFusionNode();

    void run_node();

private:
    void declare_ros_parameter();

    void set_obstacles_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr &subscriber);
    void set_radar_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::RadarScanObj>::SharedPtr &subscriber);
    void set_result_publisher(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);
    void set_marker_publisher(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &publisher);
    void set_fusion_publisher(rclcpp::Publisher<autonomy_ros2_message::msg::StereoObject>::SharedPtr &publisher);

    void obstacles_callback(const autonomy_ros2_message::msg::StereoObject::SharedPtr input);
    void radar_callback(const autonomy_ros2_message::msg::RadarScanObj::SharedPtr &input);

    void synchronize_data(const std::vector<autonomy_ros2_message::msg::RadarScanObj> radar_vec, const std::vector<autonomy_ros2_message::msg::StereoObject> pc_vec,
                          autonomy_ros2_message::msg::RadarScanObj::SharedPtr &custom_radar, autonomy_ros2_message::msg::StereoObject::SharedPtr &input_pc);

    void convert_to_point_cloud(autonomy_ros2_message::msg::StereoObject::SharedPtr &custom_radar, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc);

    void combine_obstacles(autonomy_ros2_message::msg::RadarScanObj::SharedPtr &custom_radar, autonomy_ros2_message::msg::StereoObject::SharedPtr &custom_pc, autonomy_ros2_message::msg::StereoObject::SharedPtr &result);
    void add_center_marker(const autonomy_ros2_message::msg::StereoVector vec, const int index, const rclcpp::Time timestamp, visualization_msgs::msg::MarkerArray::SharedPtr &output);
    void add_text_marker(const autonomy_ros2_message::msg::StereoVector vec, const int index, const rclcpp::Time timestamp, visualization_msgs::msg::MarkerArray::SharedPtr &output);

    void radar_fusion_thread();

    rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr m_obstacles_subscriber;
    rclcpp::Subscription<autonomy_ros2_message::msg::RadarScanObj>::SharedPtr m_radar_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_publisher;
    rclcpp::Publisher<autonomy_ros2_message::msg::StereoObject>::SharedPtr m_fusion_publisher;

    std::string m_obstacles_topic_to_subscribe;
    std::string m_radar_topic_to_subscribe;
    std::string m_point_topic_to_publish;
    std::string m_marker_topic_to_publish;
    std::string m_fusion_topic_to_publish;
    std::string m_frame_id_name;
    bool m_init_custom = false;
    bool m_init_radar = false;

    std::vector<autonomy_ros2_message::msg::RadarScanObj> m_radar_data_vec;
    std::vector<autonomy_ros2_message::msg::StereoObject> m_point_cloud_data_vec;

    std::thread m_thread;
};

#endif // __RADAR_FUSION_H__
