#ifndef __POST_FILTERING_H__
#define __POST_FILTERING_H__

#include "util/range_util.h"

constexpr int STACK_SIZE = 5;
constexpr double MAX_RANGE_METER = 6.0;
constexpr double FPS = 15.0;
constexpr double FPS_FRATION = 1.0 / FPS;

///////Obstacles Threshold///////////
constexpr double MIN_HEIGHT = 0.3;
constexpr int MIN_HEIGHT_CLUSTER_SIZE = 20;
constexpr double CLUSTER_MIN_HEIGHT = 0.1;
constexpr double UNDER_GROUND_HEIGHT = -0.1;
constexpr double UNDER_GROUND_MAX_RANGE = 4;

///////Curb Threshold////////////////
constexpr double NOISE_MAX_HEIGHT = 0.07;
constexpr int NOISE_MAX_SIZE = 10;
constexpr double NOISE_MAX_AREA = 1.2;
constexpr double NOISE_MAX_RANGE = 2.0;

class PostFilteringNode
{

public:
    PostFilteringNode(rclcpp::Node &node);

    virtual ~PostFilteringNode();

    void run_node();

private:
    void get_ros_parameter();

    void set_obstacles_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr &subscriber);
    void set_curb_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr &subscriber);
    void set_point_publisher(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);
    void set_custom_publisher(rclcpp::Publisher<autonomy_ros2_message::msg::StereoObject>::SharedPtr &publisher);

    void curb_callback(const autonomy_ros2_message::msg::StereoObject::SharedPtr &input);
    void obstacles_callback(const autonomy_ros2_message::msg::StereoObject::SharedPtr &input);

    void synchronize_data(const std::vector<autonomy_ros2_message::msg::StereoObject> detection_vec, const std::vector<autonomy_ros2_message::msg::StereoObject> pc_vec,
                          autonomy_ros2_message::msg::StereoObject::SharedPtr &custom_detection, autonomy_ros2_message::msg::StereoObject::SharedPtr &input_pc, bool &sync);

    void filter_obstacles(const autonomy_ros2_message::msg::StereoObject::SharedPtr input, autonomy_ros2_message::msg::StereoObject::SharedPtr &custom_output, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_output);
    void filter_curb(const autonomy_ros2_message::msg::StereoObject::SharedPtr input, autonomy_ros2_message::msg::StereoObject::SharedPtr &custom_output, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_output);

    void post_filtering_thread();

    rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr m_obstacles_subscriber;
    rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr m_curb_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_result_publisher;
    rclcpp::Publisher<autonomy_ros2_message::msg::StereoObject>::SharedPtr m_custom_result_publisher;

    std::string m_point_result_topic_to_publish;
    std::string m_custom_result_topic_to_publish;
    std::string m_obstacles_topic_to_subcribe;
    std::string m_curb_topic_to_subcribe;
    std::string m_frame_id_name;

    bool m_init_curb = false;
    bool m_init_obstacles = false;

    std::mutex m_curb_mutex;
    std::mutex m_obstacles_mutex;

    rclcpp::Node &m_node;

    std::vector<autonomy_ros2_message::msg::StereoObject> m_obstacles_data_vec;
    std::vector<autonomy_ros2_message::msg::StereoObject> m_curb_data_vec;

    std::thread m_thread;
};

#endif // __POST_FILTERING_H__