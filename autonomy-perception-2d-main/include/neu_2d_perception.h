

#ifndef __NEU_2D_PERCEPTION_H__
#define __NEU_2D_PERCEPTION_H__

#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include "autonomy_ros2_message/msg/neu_bbox.hpp"
#include "autonomy_ros2_message/msg/neu_bbox_point.hpp"
#include "autonomy_ros2_message/msg/neu_bbox_size.hpp"
#include "autonomy_ros2_message/msg/neu_class.hpp"
#include "autonomy_ros2_message/msg/neu_detection.hpp"
#include "autonomy_ros2_message/msg/neu_object_position.hpp"
#include "neu_perception_2d.h"
#include "neu_deepstream_core.h"
#include "neu_2d_img_subs.h"
#include "autonomy_ros2_message/msg/neu_occupancy_grid.hpp"
#include "camera_calibration/inverse_perspective_mapping.h"
#include "sensor_msgs/msg/image.hpp"

#define VISUALIZE_MASK 0

class NEU2Dperception : public NEUPerception2D
{
public:
    NEU2Dperception(rclcpp::Node &node);
    ~NEU2Dperception();
    void start_det_pipeline(const unsigned int cam_num);
    void set_parameter(SuperParameter &parameter);
    void init();
    virtual unsigned int get_add_timestamp_num();
    virtual unsigned int get_injection_timestamp_num();

    rclcpp::Publisher<autonomy_ros2_message::msg::NEUDetection>::SharedPtr m_det_publisher[MAX_CAM];
    rclcpp::Publisher<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr m_seg_publisher[MAX_CAM];

private:
    virtual GstPadProbeReturn det_pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    virtual GstPadProbeReturn seg_pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);

    autonomy_ros2_message::msg::NEUDetection m_detection[MAX_CAM];
    autonomy_ros2_message::msg::NEUBbox m_bboxes[MAX_CAM];
    autonomy_ros2_message::msg::NEUBboxPoint m_bbox_point[MAX_CAM];
    autonomy_ros2_message::msg::NEUBboxSize m_bbox_size[MAX_CAM];
    autonomy_ros2_message::msg::NEUClass m_classes[MAX_CAM];
    autonomy_ros2_message::msg::NEUObjectPosition m_obj_position[MAX_CAM];

    float m_scale_w[MAX_CAM], m_scale_h[MAX_CAM];
    std::vector<GstPadProbeReturn (*)(GstPad *, GstPadProbeInfo *, gpointer)> m_probe_vector;

    std::vector<signed char> m_class_map[MAX_CAM];
    std::vector<float> m_class_probs_map[MAX_CAM];
    autonomy_ros2_message::msg::NEUOccupancyGrid m_image_msg[MAX_CAM];
    InversePerspectiveMapping m_ipm[MAX_CAM];
    float m_mean_light;

#if VISUALIZE_MASK
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_temp_publisher2[MAX_CAM];
    sensor_msgs::msg::Image m_temp_class_map2[MAX_CAM];
#endif
};

#endif // __NEU_2D_PERCEPTION_H__
