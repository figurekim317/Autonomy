#ifndef __DETECTION_TRANSFORMER_H__
#define __DETECTION_TRANSFORMER_H__

#include <rclcpp/rclcpp.hpp>

#include <utility>

#include "image_dimension_transfomration/transformer.h"
#include "autonomy_ros2_message/msg/neu_detection.hpp"

class DetectionTransformer : public rclcpp::Node
{
public:
    DetectionTransformer(const std::string &node_name);
    ~DetectionTransformer();

    void set_detection_subs_callback(std::string &cam_name);
    void detection_sub_callback(std::string &cam_name, autonomy_ros2_message::msg::NEUDetection::SharedPtr detection_msg);

    void set_image_3d_point_publisher(std::string &cam_name);

    bool is_except_class(const int class_id);
    bool is_ground_class(const int class_id);
    bool is_proper_distance(const double distance);

private:
    std::map<std::string, rclcpp::Publisher<autonomy_ros2_message::msg::NEUDetection>::SharedPtr> m_od_publisher_map;
    std::map<std::string, rclcpp::Subscription<autonomy_ros2_message::msg::NEUDetection>::SharedPtr> m_od_subscriber_map;

    std::vector<std::string> m_except_class;
    std::vector<std::string> m_ground_class;
    std::vector<std::string> m_cam_names;
    double m_prior_height;
    double m_last_check_z_value;
    double m_check_z_linspace_value;
    int m_cam_res_height;
    int m_cam_res_width;
    bool m_is_debug;
    int m_mode;

    autonomy_ros2_message::msg::NEUDetection m_detection_msg;
    double m_max_distance_threshold;
    double m_under_principle_point_distance_thr;

    std::shared_ptr<Transformer> m_transformer;
};

#endif // __DETECTION_TRANSFORM_H__
