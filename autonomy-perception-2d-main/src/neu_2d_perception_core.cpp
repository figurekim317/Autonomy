#include <gst/app/gstappsrc.h>
#include "neu_2d_perception.h"
#include "camera_info/neu_camera_info.h"

int get_cam_num(const std::string &cam_name)
{
    if (cam_name == "cam_f")
        return static_cast<int>(CamNum::CAM_F);
    else if (cam_name == "cam_fl")
        return static_cast<int>(CamNum::CAM_FL);
    else if (cam_name == "cam_fr")
        return static_cast<int>(CamNum::CAM_FR);
    else if (cam_name == "cam_bl")
        return static_cast<int>(CamNum::CAM_BL);
    else if (cam_name == "cam_br")
        return static_cast<int>(CamNum::CAM_BR);
}

void extract_common_cam(std::vector<std::string> &det_cam, std::vector<std::string> &seg_cam, std::vector<std::string> &common_cam)
{
    for (int i = 0; i < det_cam.size(); ++i)
    {
        if (seg_cam.end() != std::find(seg_cam.begin(), seg_cam.end(), det_cam[i]))
            common_cam.push_back(det_cam[i]);
    }

    for (int i = 0; i < det_cam.size(); ++i)
    {
        if (common_cam.end() != std::find(common_cam.begin(), common_cam.end(), det_cam[i]))
            det_cam.erase(std::remove(det_cam.begin(), det_cam.end(), det_cam[i]), det_cam.end());
    }

    for (int i = 0; i < seg_cam.size(); ++i)
    {
        if (common_cam.end() != std::find(common_cam.begin(), common_cam.end(), seg_cam[i]))
            seg_cam.erase(std::remove(seg_cam.begin(), seg_cam.end(), seg_cam[i]), seg_cam.end());
    }
}

void set_param_resolution(SuperParameter &parameter, const std::string &cam_name)
{
    parameter.set_video_width(NeuCameraInfo::get_instance().get_camera_width(cam_name));
    parameter.set_video_height(NeuCameraInfo::get_instance().get_camera_height(cam_name));
    parameter.set_input_width(NeuCameraInfo::get_instance().get_model_width(cam_name));
    parameter.set_input_height(NeuCameraInfo::get_instance().get_model_height(cam_name));
    parameter.set_crop_size(NeuCameraInfo::get_instance().get_crop(cam_name));
}

void NEU2Dperception::init()
{
    m_parameter = new Perception2dParameter();
    for (size_t i = 0; i < MAX_CAM; ++i)
    {
        m_obj_position[i].x = -1;
        m_obj_position[i].y = -1;
        m_obj_position[i].z = -1;
        m_classes[i].attribute = 0;
    }
    set_parameter(*m_parameter);
    m_probe_vector.push_back(add_timestamp);
    m_probe_vector.push_back(inject_timestamp);
    m_probe_vector.push_back(det_pad_buffer_probe);
    m_probe_vector.push_back(seg_pad_buffer_probe);

    m_checker_ready = true;
    m_sys_info_publisher = m_node.create_publisher<std_msgs::msg::String>(m_task + "_system_info", 10);
    m_thread_pool.push_back(std::thread(&NEU2Dperception::system_info_thread, this));

    std::vector<std::string> det_cam_list = m_parameter->get_cam_list();
    std::vector<std::string> seg_cam_list = m_parameter->get_cam_list2();

    std::vector<std::string> common_pipeline;
    int idx = 0;

    extract_common_cam(det_cam_list, seg_cam_list, common_pipeline);

    for (int i = 0; i < common_pipeline.size(); ++i)
    {
        idx = m_perception[i]->get_cam_num(common_pipeline[i]);
        if (idx == -1)
            continue;
        m_subs[idx] = new SubsriberImage(m_perception[i]->get_cam_name(idx));
        m_det_publisher[idx] = m_node.create_publisher<autonomy_ros2_message::msg::NEUDetection>(common_pipeline[i] + "_object_detection", 10);
        m_seg_publisher[idx] = m_node.create_publisher<autonomy_ros2_message::msg::NEUOccupancyGrid>(common_pipeline[i] + "_semantic_segmentation", 10);
#if VISUALIZE_MASK
        m_temp_publisher2[idx] = m_node.create_publisher<sensor_msgs::msg::Image>(common_pipeline[i] + "_undistrot_class_map", 10);
#endif
        set_param_resolution(*m_parameter, common_pipeline[i]);

        m_ipm[get_cam_num(common_pipeline[i])] = InversePerspectiveMapping(common_pipeline[i]);

        m_scale_w[get_cam_num(common_pipeline[i])] = (m_parameter->get_video_width() - m_parameter->get_crop_size()[0]) / (float)NeuCameraInfo::get_instance().get_model_width(common_pipeline[i]);
        m_scale_h[get_cam_num(common_pipeline[i])] = (m_parameter->get_video_height() - m_parameter->get_crop_size()[1]) / (float)NeuCameraInfo::get_instance().get_model_height(common_pipeline[i]);

        std::vector<std::string> det_cam, seg_cam;
        det_cam.push_back(common_pipeline[i]);
        seg_cam.push_back(common_pipeline[i]);
        m_parameter->set_cam_list(det_cam);
        m_parameter->set_cam_list2(seg_cam);
        m_parameter->set_task_name("Dual");

        m_mutex.lock();
        std::pair<GstElement *, GMainLoop *> pipe = m_perception[i]->set_pipeline(m_probe_vector, *m_parameter, this);

        m_loop[idx] = pipe.second;
        m_pipeline[idx] = pipe.first;
        gst_element_set_state(m_pipeline[idx], GST_STATE_PLAYING);
        RCLCPP_WARN(m_node.get_logger(), "Make Gst Pipeline Done");
        m_mutex.unlock();
        m_thread_pool.push_back(std::thread(&NEU2Dperception::start_det_pipeline, this, idx));

        this->get_subs(idx)->set_subscriber(this->get_node_());
    }

    for (int i = 0; i < det_cam_list.size(); ++i)
    {
        idx = m_perception[i]->get_cam_num(det_cam_list[i]);
        if (idx == -1)
            break;
        m_subs[idx] = new SubsriberImage(m_perception[i]->get_cam_name(idx));
        m_det_publisher[idx] = m_node.create_publisher<autonomy_ros2_message::msg::NEUDetection>(det_cam_list[i] + "_object_detection", 10);
#if VISUALIZE_MASK
        m_temp_publisher2[idx] = m_node.create_publisher<sensor_msgs::msg::Image>(det_cam_list[i] + "_undistrot_class_map", 10);
#endif

        set_param_resolution(*m_parameter, det_cam_list[i]);

        m_scale_w[get_cam_num(det_cam_list[i])] = (m_parameter->get_video_width() - m_parameter->get_crop_size()[0]) / (float)NeuCameraInfo::get_instance().get_model_width(det_cam_list[i]);
        m_scale_h[get_cam_num(det_cam_list[i])] = (m_parameter->get_video_height() - m_parameter->get_crop_size()[1]) / (float)NeuCameraInfo::get_instance().get_model_height(det_cam_list[i]);

        std::vector<std::string> det_cam;
        det_cam.push_back(det_cam_list[i]);
        m_parameter->set_cam_list(det_cam);
        m_parameter->set_task_name("Detection");

        m_mutex.lock();
        std::pair<GstElement *, GMainLoop *> pipe = m_perception[i]->set_pipeline(m_probe_vector, *m_parameter, this);
        m_loop[idx] = pipe.second;
        m_pipeline[idx] = pipe.first;
        gst_element_set_state(m_pipeline[idx], GST_STATE_PLAYING);
        RCLCPP_WARN(m_node.get_logger(), "Make Gst Pipeline Done");
        m_mutex.unlock();
        m_thread_pool.push_back(std::thread(&NEU2Dperception::start_det_pipeline, this, idx));

        this->get_subs(idx)->set_subscriber(this->get_node_());
    }

    for (int i = 0; i < seg_cam_list.size(); ++i)
    {
        idx = m_perception[i]->get_cam_num(seg_cam_list[i]);
        if (idx == -1)
            break;
        m_subs[idx] = new SubsriberImage(m_perception[i]->get_cam_name(idx));
        m_seg_publisher[idx] = m_node.create_publisher<autonomy_ros2_message::msg::NEUOccupancyGrid>(seg_cam_list[i] + "_semantic_segmentation", 10);
#if VISUALIZE_MASK
        m_temp_publisher2[idx] = m_node.create_publisher<sensor_msgs::msg::Image>(seg_cam_list[i] + "_undistrot_class_map", 10);
#endif
        set_param_resolution(*m_parameter, seg_cam_list[i]);

        m_ipm[get_cam_num(common_pipeline[i])] = InversePerspectiveMapping(seg_cam_list[i]);

        std::vector<std::string> seg_cam;
        seg_cam.push_back(seg_cam_list[i]);
        m_parameter->set_cam_list2(seg_cam);
        m_parameter->set_task_name("Segmentation");

        m_mutex.lock();
        std::pair<GstElement *, GMainLoop *> pipe = m_perception[i]->set_pipeline(m_probe_vector, *m_parameter, this);
        m_loop[idx] = pipe.second;
        m_pipeline[idx] = pipe.first;
        gst_element_set_state(m_pipeline[idx], GST_STATE_PLAYING);
        RCLCPP_WARN(m_node.get_logger(), "Make Gst Pipeline Done");
        m_mutex.unlock();
        m_thread_pool.push_back(std::thread(&NEU2Dperception::start_det_pipeline, this, idx));
        this->get_subs(idx)->set_subscriber(this->get_node_());
    }

    std::cout << "-- dual perception2d cam list --" << std::endl;
    for (int i = 0; i < common_pipeline.size(); ++i)
        std::cout << common_pipeline[i] << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "-- solo detection cam list --" << std::endl;
    for (int i = 0; i < det_cam_list.size(); ++i)
    {
        if (common_pipeline.end() == std::find(common_pipeline.begin(), common_pipeline.end(), det_cam_list[i]))
            std::cout << det_cam_list[i] << std::endl;
    }
    std::cout << "-------------------" << std::endl;

    std::cout << "-- solo segmentation cam list --" << std::endl;
    for (int i = 0; i < seg_cam_list.size(); ++i)
    {
        if (common_pipeline.end() == std::find(common_pipeline.begin(), common_pipeline.end(), seg_cam_list[i]))
            std::cout << seg_cam_list[i] << std::endl;
    }
    std::cout << "-------------------" << std::endl;
}

NEU2Dperception::NEU2Dperception(rclcpp::Node &node)
    : NEUPerception2D(node)
{
    m_task = "perception_2d";
    init_frame_id("Detection");
    init_frame_id("Segmentation");
    init();
}

void NEU2Dperception::set_parameter(SuperParameter &parameter)
{
    if (!m_node.has_parameter("det_config_file_name"))
    {
        m_node.declare_parameter("det_config_file_name", "integration_test.txt");
    }
    if (!m_node.has_parameter("seg_config_file_name"))
    {
        m_node.declare_parameter("seg_config_file_name", "integration_test.txt");
    }
    if (!m_node.has_parameter("det_cam_list"))
    {
        m_node.declare_parameter("det_cam_list", std::vector<std::string>{""});
    }
    if (!m_node.has_parameter("seg_cam_list"))
    {
        m_node.declare_parameter("seg_cam_list", std::vector<std::string>{""});
    }

    if (!m_node.has_parameter("tracker_config_file_name"))
        m_node.declare_parameter("tracker_config_file_name", "object_tracker_config.txt");
    if (!m_node.has_parameter("use_shared_memory"))
        m_node.declare_parameter("use_shared_memory", false);

    parameter.set_tracker_config_name(m_node.get_parameter("tracker_config_file_name").as_string());
    parameter.set_check_shm(m_node.get_parameter("use_shared_memory").as_bool());

    parameter.set_cam_list(m_node.get_parameter("det_cam_list").as_string_array());
    parameter.set_cam_list2(m_node.get_parameter("seg_cam_list").as_string_array());
    parameter.set_model_config_name(m_node.get_parameter("det_config_file_name").as_string());
    parameter.set_model_config_name2(m_node.get_parameter("seg_config_file_name").as_string());
}

void NEU2Dperception::start_det_pipeline(const unsigned int cam_num)
{
    if (m_parameter == nullptr)
        return;

    g_main_loop_run(m_loop[cam_num]);
}

GstPadProbeReturn NEU2Dperception::det_pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data)
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
    if (m_parameter->get_check_shm())
    {
        time.sec = (*(TIME_STAMP *)u_data).sec;
        time.nanosec = (*(TIME_STAMP *)u_data).nanosec;
    }
    else
    {
        if (!m_det_timestamp[cam_num].empty())
        {
            time = m_det_timestamp[cam_num].front();
            m_det_timestamp[cam_num].pop();
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

            m_bbox_size[cam_num].width = obj_meta->rect_params.width * m_scale_w[cam_num];
            m_bbox_size[cam_num].height = obj_meta->rect_params.height * m_scale_h[cam_num];
            m_bbox_point[cam_num].u = obj_meta->rect_params.left * m_scale_w[cam_num] + m_parameter->get_crop_size()[0] + m_bbox_size[cam_num].width / 2;
            m_bbox_point[cam_num].v = obj_meta->rect_params.top * m_scale_h[cam_num] + m_parameter->get_crop_size()[1] + m_bbox_size[cam_num].height / 2;

            m_classes[cam_num].id = obj_meta->class_id;
            m_classes[cam_num].confidence = obj_meta->confidence;

            m_bboxes[cam_num].id = std::to_string(obj_meta->object_id);
            m_bboxes[cam_num].status = obj_meta->class_id > 5 ? 1 : 2;
            m_bboxes[cam_num].classes = m_classes[cam_num];

            m_bboxes[cam_num].point = m_bbox_point[cam_num];
            m_bboxes[cam_num].size = m_bbox_size[cam_num];
            m_bboxes[cam_num].position = m_obj_position[cam_num];

            m_detection[cam_num].bboxes.push_back(m_bboxes[cam_num]);
        }
        m_det_publisher[cam_num]->publish(m_detection[cam_num]);
    }
    return GST_PAD_PROBE_OK;
}

GstPadProbeReturn NEU2Dperception::seg_pad_buffer_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer u_data)
{
    unsigned int cam_num = (*(TIME_STAMP *)u_data).cam_num;
    GstBuffer *gst_buffer = (GstBuffer *)info->data;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(gst_buffer);
    if (!batch_meta)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Batch meta not found for buffer");
        return GST_PAD_PROBE_OK;
    }

    NvDsMetaList *l_frame;
    NvDsUserMetaList *l_user;
    NvDsUserMeta *seg_user_meta;
    NvDsFrameMeta *frame_meta;
    NvDsInferSegmentationMeta *user_seg_data;
    guint width, height;
    int *class_map = nullptr;
    float *class_probs = nullptr;
    unsigned int plane, idx;

    builtin_interfaces::msg::Time time;
    if (m_parameter->get_check_shm())
    {
        GstCaps *ts_ref = gst_caps_new_empty_simple("TimeStampMetaData");
        GstReferenceTimestampMeta *ts = gst_buffer_get_reference_timestamp_meta(gst_buffer, ts_ref);
        if (ts != NULL)
        {
            time.sec = ts->timestamp;
            time.nanosec = ts->duration;
        }

        gst_caps_unref(ts_ref);
    }
    else
    {
        if (!m_seg_timestamp[cam_num].empty())
        {
            time = m_seg_timestamp[cam_num].front();
            m_seg_timestamp[cam_num].pop();
        }
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame != nullptr; l_frame = l_frame->next)
    {
        frame_meta = (NvDsFrameMeta *)(l_frame->data);
        for (l_user = frame_meta->frame_user_meta_list; l_user != nullptr; l_user = l_user->next)
        {
            seg_user_meta = (NvDsUserMeta *)(l_user->data);
            if (seg_user_meta != nullptr && seg_user_meta->base_meta.meta_type == NVDSINFER_SEGMENTATION_META)
            {
                user_seg_data = (NvDsInferSegmentationMeta *)(seg_user_meta->user_meta_data);
                width = user_seg_data->width;
                height = user_seg_data->height;
                class_map = user_seg_data->class_map;
                class_probs = user_seg_data->class_probabilities_map;

                m_image_msg[cam_num].header.frame_id = m_frame_id[cam_num + MAX_CAM];
                m_image_msg[cam_num].header.stamp = time;

                m_image_msg[cam_num].light_value = m_mean_light;
                m_image_msg[cam_num].info.width = width;
                m_image_msg[cam_num].info.height = height;
                m_image_msg[cam_num].info.resolution = 0.1;

                plane = width * height;
                if (m_class_probs_map[cam_num].size() != plane)
                    m_class_probs_map[cam_num].resize(plane);
                for (idx = 0; idx < plane; idx++)
                {
                    m_class_probs_map[cam_num][idx] = class_probs[idx + class_map[idx] * plane];
                }
                m_image_msg[cam_num].data = m_class_probs_map[cam_num];

                m_ipm[cam_num].undistort(class_map, width, height, m_parameter->get_crop_size(), m_class_map[cam_num]);
                m_image_msg[cam_num].classes = m_class_map[cam_num];
                m_seg_publisher[cam_num]->publish(m_image_msg[cam_num]);
#if VISUALIZE_MASK
                std::vector<unsigned char> ipm2;
                ipm2.reserve(plane * 3);
                for (int i = 0; i < plane; i++)
                {
                    if (class_map[i] == 0)
                    {
                        ipm2.push_back(0);
                        ipm2.push_back(0);
                        ipm2.push_back(0);
                    }
                    else if (class_map[i] == 1)
                    {
                        ipm2.push_back(255);
                        ipm2.push_back(0);
                        ipm2.push_back(0);
                    }
                    else if (class_map[i] == 2)
                    {
                        ipm2.push_back(0);
                        ipm2.push_back(255);
                        ipm2.push_back(0);
                    }
                    else if (class_map[i] == 3)
                    {
                        ipm2.push_back(0);
                        ipm2.push_back(0);
                        ipm2.push_back(255);
                    }
                }
                m_temp_class_map2[cam_num].width = width;
                m_temp_class_map2[cam_num].height = height;
                m_temp_class_map2[cam_num].step = width * 3;
                m_temp_class_map2[cam_num].encoding = "rgb8";

                m_temp_class_map2[cam_num].data = ipm2;
                m_temp_publisher2[cam_num]->publish(m_temp_class_map2[cam_num]);
#endif
            }
        }
    }
    return GST_PAD_PROBE_OK;
}

unsigned int NEU2Dperception::get_add_timestamp_num()
{
    return static_cast<int>(Perception2d::ADD_TIMESTAMP);
}
unsigned int NEU2Dperception::get_injection_timestamp_num()
{
    return static_cast<int>(Perception2d::INJECTION_TIMESTAMP);
}

NEU2Dperception::~NEU2Dperception()
{
    if (m_parameter != nullptr)
        delete m_parameter;
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

        if (m_loop[i] != nullptr)
        {
            g_main_destroy(m_loop[i]);
            m_loop[i] = nullptr;
        }
    }
}
