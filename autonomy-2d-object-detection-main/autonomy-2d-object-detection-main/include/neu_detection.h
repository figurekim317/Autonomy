#ifndef __NEU_DETECTION_H__
#define __NEU_DETECTION_H__

#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include "autonomy_ros2_message/msg/neu_bbox.hpp"

#include "autonomy_ros2_message/msg/neu_bbox_size.hpp"
#include "autonomy_ros2_message/msg/neu_class.hpp"
#include "autonomy_ros2_message/msg/neu_detection.hpp"
#include "autonomy_ros2_message/msg/neu_object_position.hpp"
#include "neu_perception_2d.h"
#include "neu_deepstream_core.h"
#include "neu_2d_img_subs.h"

class NEUDetection : public NEUPerception2D
{
public:
    NEUDetection(rclcpp::Node &node);
    ~NEUDetection();
    void start_det_pipeline(const unsigned int cam_num);
    void set_parameter(Parameter &parameter);
    void init();

    virtual unsigned int get_task_num();
    virtual unsigned int get_add_timestamp_num();
    virtual unsigned int get_injection_timestamp_num();

    rclcpp::Publisher<autonomy_ros2_message::msg::NEUDetection>::SharedPtr m_publisher[MAX_CAM];

private:
    virtual GstPadProbeReturn pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);

    autonomy_ros2_message::msg::NEUDetection m_detection[MAX_CAM];
    autonomy_ros2_message::msg::NEUBbox m_bboxes[MAX_CAM];
    autonomy_ros2_message::msg::NEUBboxPoint m_bbox_point[MAX_CAM];
    autonomy_ros2_message::msg::NEUBboxSize m_bbox_size[MAX_CAM];
    autonomy_ros2_message::msg::NEUClass m_classes[MAX_CAM];
    autonomy_ros2_message::msg::NEUObjectPosition m_obj_position[MAX_CAM];

    float m_scale_w, m_scale_h;
    std::mutex m_mutex;
    std::vector<GstPadProbeReturn (*)(GstPad *, GstPadProbeInfo *, gpointer)> m_probe_vector;
};

#endif //__NEU_DETECTION_H__
