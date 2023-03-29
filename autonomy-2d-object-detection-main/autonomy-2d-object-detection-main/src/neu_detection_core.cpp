#include <gst/app/gstappsrc.h>
#include "neu_detection.h"

constexpr int CLASS_NUM = 10;
constexpr double BBOX_COLORS[CLASS_NUM][4] = {
    {0.5, 0.5, 0, 1},
    {0, 0.5, 0.5, 1},
    {0.5, 0, 0.5, 1},
    {0.5, 0.2, 0.2, 1},
    {0.2, 0.5, 0.2, 1},
    {0.2, 0.2, 0.5, 1},
    {0, 0, 1, 1},
    {1, 0, 0, 1},
    {0, 1, 0, 1},
    {1, 0, 1, 1},
};

void NEUDetection::init()
{
    for (size_t i = 0; i < MAX_CAM; ++i)
    {
        m_obj_position[i].x = -1;
        m_obj_position[i].y = -1;
        m_obj_position[i].z = -1;
        m_classes[i].attribute = 0;
        m_bboxes[i].classes = std::vector<autonomy_ros2_message::msg::NEUClass>(1);

        m_parameter[i] = new DetectionParameter();
        set_parameter(*m_parameter[i]);

        m_scale_w = m_parameter[i]->get_video_width() / (float)m_parameter[i]->get_input_width();
        m_scale_h = m_parameter[i]->get_video_height() / (float)m_parameter[i]->get_input_height();
    }

    m_checker_ready = true;
    m_sys_info_publisher = m_node.create_publisher<std_msgs::msg::String>(m_task + "_system_info", 10);
    m_thread_pool.push_back(std::thread(&NEUDetection::system_info_thread, this));

    std::vector<std::string> cam_list = m_parameter[0]->get_cam_list();
    std::vector<int> live_cam_list;
    int idx = 0;
    for (size_t i = 0; i < cam_list.size(); ++i)
    {
        idx = m_perception[i]->get_cam_num(cam_list[i]);
        m_subs[idx] = new SubsriberImage(m_perception[i]->get_cam_name(idx));
        m_publisher[idx] = m_node.create_publisher<autonomy_ros2_message::msg::NEUDetection>(cam_list[i] + "_object_detection", 10);
        std::vector<std::string> temp_cam_list;
        temp_cam_list.clear();
        temp_cam_list.push_back(cam_list[i]);
        m_parameter[idx]->set_cam_list(temp_cam_list);
        for (size_t j = 0; j < temp_cam_list.size(); ++j)
            std::cout << temp_cam_list[j] << std::endl;
        live_cam_list.push_back(idx);
    }

    m_probe_vector.push_back(pad_buffer_probe);
    m_probe_vector.push_back(add_timestamp);
    m_probe_vector.push_back(inject_timestamp);
    for (size_t i = 0; i < MAX_CAM; ++i)
    {
        if (std::find(live_cam_list.begin(), live_cam_list.end(), i) == live_cam_list.end())
        {
            delete m_parameter[i];
            m_parameter[i] = nullptr;
        }
        else
        {
            m_mutex.lock();
            std::pair<GstElement *, GMainLoop *> pipe = m_perception[i]->set_legacy_pipeline(m_probe_vector, *m_parameter[i], this);
            m_loop[i] = pipe.second;
            m_pipeline[i] = pipe.first;
            gst_element_set_state(m_pipeline[i], GST_STATE_PLAYING);
            RCLCPP_WARN(m_node.get_logger(), "Make Gst Pipeline Done");
            m_mutex.unlock();
            m_thread_pool.push_back(std::thread(&NEUDetection::start_det_pipeline, this, i));
        }
    }
}

NEUDetection::NEUDetection(rclcpp::Node &node)
    : NEUPerception2D(node)
{
    m_task = "detection";
    init_frame_id();
    init();
}

void NEUDetection::set_parameter(Parameter &parameter)
{
    if (!m_node.has_parameter("model_config_file_name"))
        m_node.declare_parameter("model_config_file_name", "yolov5_multicam_infer_cls10_batch1.txt");
    if (!m_node.has_parameter("tracker_config_file_name"))
        m_node.declare_parameter("tracker_config_file_name", "object_tracker_config.txt");
    if (!m_node.has_parameter("crop_size"))
        m_node.declare_parameter<std::vector<int64_t>>("crop_size", std::vector<int64_t>{0, 0, 2592, 1944});
    if (!m_node.has_parameter("input_width"))
        m_node.declare_parameter("input_width", 2592);
    if (!m_node.has_parameter("input_height"))
        m_node.declare_parameter("input_height", 1944);
    if (!m_node.has_parameter("cam_list"))
        m_node.declare_parameter<std::vector<std::string>>("cam_list", std::vector<std::string>{"cam_f"});
    if (!m_node.has_parameter("video_width"))
        m_node.declare_parameter("video_width", 2592);
    if (!m_node.has_parameter("video_height"))
        m_node.declare_parameter("video_height", 1944);
    if (!m_node.has_parameter("use_shared_memory"))
        m_node.declare_parameter("use_shared_memory", false);

    parameter.set_model_config_name(m_node.get_parameter("model_config_file_name").as_string());
    parameter.set_tracker_config_name(m_node.get_parameter("tracker_config_file_name").as_string());
    parameter.set_crop_size(m_node.get_parameter("crop_size").as_integer_array());
    parameter.set_video_size(m_node.get_parameter("video_width").as_int(), m_node.get_parameter("video_height").as_int());
    parameter.set_input_size(m_node.get_parameter("input_width").as_int(), m_node.get_parameter("input_height").as_int());
    parameter.set_cam_list(m_node.get_parameter("cam_list").as_string_array());
    parameter.set_check_shm(m_node.get_parameter("use_shared_memory").as_bool());
}

void NEUDetection::start_det_pipeline(const unsigned int cam_num)
{
    if (m_parameter[cam_num] == nullptr)
        return;

    g_main_loop_run(m_loop[cam_num]);
}

GstPadProbeReturn NEUDetection::pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data)
{
    GstBuffer *gst_buffer = (GstBuffer *)info->data;
    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(gst_buffer);
    if (!batch_meta)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Batch meta not found for buffer");
        return GST_PAD_PROBE_OK;
    }
    NvDsMetaList *l_frame;
    NvDsMetaList *l_obj;
    NvDsObjectMeta *obj_meta;
    NvDsFrameMeta *frame_meta;

    unsigned int cam_num = (*(TIME_STAMP *)u_data).cam_num;

    builtin_interfaces::msg::Time time;
    if (m_parameter[cam_num]->get_check_shm())
    {
        time.sec = (*(TIME_STAMP *)u_data).sec;
        time.nanosec = (*(TIME_STAMP *)u_data).nanosec;
    }
    else
    {
        if (!m_timestamp[cam_num].empty())
        {
            time = m_timestamp[cam_num].front();
            m_timestamp[cam_num].pop();
        }
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame != nullptr; l_frame = l_frame->next)
    {
        frame_meta = (NvDsFrameMeta *)(l_frame->data);
        m_detection[cam_num].header.frame_id = m_frame_id[cam_num];

        m_detection[cam_num].header.stamp = time;
        m_detection[cam_num].bboxes.clear();

        for (l_obj = frame_meta->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next)
        {
            obj_meta = (NvDsObjectMeta *)(l_obj->data);

            m_bbox_size[cam_num].width = obj_meta->rect_params.width * m_scale_w;
            m_bbox_size[cam_num].height = obj_meta->rect_params.height * m_scale_h;
            m_bbox_point[cam_num].u = obj_meta->rect_params.left * m_scale_w + m_parameter[cam_num]->get_crop_size()[0] + m_bbox_size[cam_num].width / 2;
            m_bbox_point[cam_num].v = obj_meta->rect_params.top * m_scale_h + m_parameter[cam_num]->get_crop_size()[1] + m_bbox_size[cam_num].height / 2;

            m_classes[cam_num].id = std::to_string(obj_meta->class_id);
            m_classes[cam_num].confidence = obj_meta->confidence;

            m_bboxes[cam_num].id = std::to_string(obj_meta->object_id);
            m_bboxes[cam_num].status = obj_meta->class_id > 5 ? 1 : 2;
            m_bboxes[cam_num].classes[0] = m_classes[cam_num];

            m_bboxes[cam_num].point = m_bbox_point[cam_num];
            m_bboxes[cam_num].size = m_bbox_size[cam_num];
            m_bboxes[cam_num].position = m_obj_position[cam_num];

            m_detection[cam_num].bboxes.push_back(m_bboxes[cam_num]);
        }
        m_publisher[cam_num]->publish(m_detection[cam_num]);
    }
    return GST_PAD_PROBE_OK;
}

unsigned int NEUDetection::get_task_num()
{
    return static_cast<int>(DetectionRole::PUBLISH_RESULT_MESSAGE);
}
unsigned int NEUDetection::get_add_timestamp_num()
{
    return static_cast<int>(DetectionRole::ADD_TIMESTAMP);
}
unsigned int NEUDetection::get_injection_timestamp_num()
{
    return static_cast<int>(DetectionRole::INJECTION_TIMESTAMP);
}

NEUDetection::~NEUDetection()
{

    for (size_t i = 0; i < MAX_CAM; ++i)
    {
        if (m_subs[i] != nullptr)
        {
            delete m_subs[i];
            m_subs[i] = nullptr;
        }
        if (m_pipeline[i] != nullptr)
        {
            gst_element_send_event(m_pipeline[i], gst_event_new_eos());
            gst_element_set_state(m_pipeline[i], GST_STATE_NULL);
            gst_object_unref(m_pipeline[i]);
            delete m_pipeline[i];
            m_pipeline[i] = nullptr;
        }

        if (m_parameter[i] != nullptr)
        {
            delete m_parameter[i];
            m_parameter[i] = nullptr;
        }
        if (m_loop[i] != nullptr)
        {
            g_main_destroy(m_loop[i]);
            m_loop[i] = nullptr;
        }
    }

    for (size_t i = 0; i < m_thread_pool.size(); i++)
    {
        if (m_thread_pool[i].joinable())
        {
            m_thread_pool.at(i).join();
        }
    }
}
