#ifndef __NEU_PERCEPTION_2D_H__
#define __NEU_PERCEPTION_2D_H__

#include <thread>
#include <queue>
#include "pipeline/perception_2d_parameter.h"
#include "nvbufsurface.h"
#include "gstnvdsmeta.h"
#include "gstnvdsinfer.h"
#include "neu_2d_img_subs.h"
#include "std_msgs/msg/string.hpp"
#include "system_checker/neu_system_checker.h"

class NEUPerception2D
{
public:
    NEUPerception2D(rclcpp::Node &node);
    virtual ~NEUPerception2D();
    SuperParameter *get_parameter() const;
    static GstPadProbeReturn calc_brightness_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    static GstPadProbeReturn det_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    static GstPadProbeReturn seg_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);

    static GstPadProbeReturn add_timestamp(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    static GstPadProbeReturn inject_timestamp(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    SubsriberImage *get_subs(const unsigned int cam_num) const;
    rclcpp::Node &get_node_() const;

    virtual unsigned int get_add_timestamp_num();
    virtual unsigned int get_injection_timestamp_num();

    void init_frame_id();
    void init_frame_id(const std::string &task_name);
    virtual void system_info_thread();

protected:
    virtual GstPadProbeReturn calculate_brightness_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    virtual GstPadProbeReturn det_pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    virtual GstPadProbeReturn seg_pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    virtual GstPadProbeReturn add_timestamp_meta(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    virtual GstPadProbeReturn inject_timestamp_meta(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);

    std::queue<builtin_interfaces::msg::Time> m_det_timestamp[MAX_CAM];
    std::queue<builtin_interfaces::msg::Time> m_seg_timestamp[MAX_CAM];
    SubsriberImage *m_subs[MAX_CAM];
    std::vector<std::string> m_frame_id;
    rclcpp::Node &m_node;
    GstElement *m_pipeline[MAX_CAM];
    GMainLoop *m_loop[MAX_CAM];
    SuperParameter *m_parameter;
    std::vector<std::thread> m_thread_pool;

    NEUPerception *m_perception[MAX_CAM];
    std::string m_task;
    std::mutex m_mutex;
    bool m_checker_ready;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_sys_info_publisher;
};

#endif // __NEU_PERCEPTION_2D_H__
