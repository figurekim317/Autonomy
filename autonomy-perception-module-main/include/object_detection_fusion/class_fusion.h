#ifndef __OBJECT_FUSION_H__
#define __OBJECT_FUSION_H__

#include <autonomy_ros2_message/msg/neu_detection.hpp>

#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <range_util.h>

constexpr int STACK_SIZE = 3;
constexpr double WIDTH = 2592.0;
constexpr double HEIGHT = 1944.0;
constexpr double FPS = 15.0;
constexpr double FPS_FRATION = 1.0 / FPS;
constexpr double ROBOT_HEIGHT = -0.575;
constexpr double METER_PER_PIXEL = 1.0;
constexpr double CLOSE_POINT = 1.0;
constexpr double FAR_POINT = 6.0;
constexpr double TARGET_WIDTH = 0.3;

class ClassFusionNode : public rclcpp::Node
{
public:
    ClassFusionNode(std::string node_name);

    virtual ~ClassFusionNode();

    void run_node();

    Eigen::Matrix3d get_camera_matrix() const;
    Eigen::Matrix3d get_perspective_matrix() const;

private:
    void declare_ros_parameter();

    void set_custom_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr &subscriber);
    void set_detection_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUDetection>::SharedPtr &subscriber);
    void set_result_publisher(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);

    Eigen::Matrix3d calculate_perspective_transform();
    void set_camera_parameter(const std::string cam_name);

    void obstacles_callback(const autonomy_ros2_message::msg::StereoObject::SharedPtr input);
    void detection_callback(const autonomy_ros2_message::msg::NEUDetection::SharedPtr &input);

    void synchronize_data(const std::vector<autonomy_ros2_message::msg::NEUDetection> detection_vec, const std::vector<autonomy_ros2_message::msg::StereoObject> pc_vec,
                          autonomy_ros2_message::msg::NEUDetection::SharedPtr &custom_detection, autonomy_ros2_message::msg::StereoObject::SharedPtr &input_pc);

    void project_to_image(const autonomy_ros2_message::msg::StereoObject::SharedPtr obs, autonomy_ros2_message::msg::NEUDetection::SharedPtr &pc_det);

    void class_fusion_thread();
    rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr m_custom_subscriber;
    rclcpp::Subscription<autonomy_ros2_message::msg::NEUDetection>::SharedPtr m_detection_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_msg_publisher;

    std::string m_msg_topic_to_publish;
    std::string m_custom_topic_to_subscribe;
    std::string m_detection_topic_to_subscribe;
    std::string m_frame_id_name;
    bool m_init_custom = false;
    bool m_init_detection = false;
    Eigen::Matrix3d m_distortion_correction_matrix;
    Eigen::Matrix3d m_perspective_matrix;

    std::vector<autonomy_ros2_message::msg::NEUDetection> m_detection_data_vec;
    std::vector<autonomy_ros2_message::msg::StereoObject> m_point_cloud_data_vec;

    std::thread m_thread;
};

#endif // __OBJECT_FUSION_H__
