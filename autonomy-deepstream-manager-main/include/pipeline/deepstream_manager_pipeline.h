#ifndef __DEEPSTREAM_MANAGER_PIPELINE_H__
#define __DEEPSTREAM_MANAGER_PIPELINE_H__

#include <map>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include "pipeline/pipeline_parameter.h"
#include "rclcpp/rclcpp.hpp"

// class NEUPerception2D;

class DeepstreamManager
{
public:
    GstElement *make_gst_pipeline(const char *name);
    GstElement *make_gst_factory(const char *factory_name, const char *name);
    GstElement *make_gst_appsrc(const char *name, unsigned int width, unsigned int height);
    GstElement *make_gst_jpeg_decoder(const char *name);
    GstElement *make_gst_caps_filter(const char *name, const SuperParameter &param);
    GstElement *make_gst_streammux(const char *name, const unsigned int batch_size, const SuperParameter &param);
    GstElement *make_gst_pgie(const char *name, const unsigned int batch_size, const SuperParameter &param);
    GstElement *make_gst_nvvideoconvert_in(const char *name, const SuperParameter &param);
    GstElement *make_gst_shmsrc(const char *name, const char *socket_path);
    GstElement *make_gst_tracker(const char *name, const SuperParameter &param);
    void read_config(const std::string &path, std::map<std::string, std::string> &config_map);
    void get_config(const std::string &sentence, std::map<std::string, std::string> &config_map);
    void set_tracker_config(GstElement *element, const std::map<std::string, std::string> &config_map);

    // static DeepstreamManager &get_instance();

private:
};

#endif //__DEEPSTREAM_MANAGER_PIPELINE_H__
