#ifndef __NEU_DEEPSTREAM_CORE_H__
#define __NEU_DEEPSTREAM_CORE_H__

#include "pipeline/deepstream_manager_pipeline.h"

typedef struct
{
    void *data;
    int cam_num;
} PERCEPTION_INFO;

class NEUPerception : public DeepstreamManager
{

public:
    GstElement *essential_pipeline(const std::vector<GstPadProbeReturn (*)(GstPad *pad, GstPadProbeInfo *info, gpointer u_data)> &probe_vector, const SuperParameter &param, void *task, GstElement *pipeline, GstElement *element);
    std::pair<GstElement *, GMainLoop *> set_pipeline(const std::vector<GstPadProbeReturn (*)(GstPad *pad, GstPadProbeInfo *info, gpointer u_data)> &probe_vector, const SuperParameter &param, void *task);
    static gboolean bus_call(__attribute__((unused)) GstBus *bus, GstMessage *msg, gpointer data);

    std::string get_cam_name(const unsigned int cam_num);
    unsigned int get_cam_num(const std::string &cam);
    static unsigned int m_pipeline_idx;
    void push_buf(GstElement *appsrc, GstBuffer *buf, const unsigned int cam_num);
    virtual ~NEUPerception();

private:
    NEUPerception();
};

#endif //__NEU_DEEPSTREAM_CORE_H__
