#ifndef __PIPELINE_PARAMETER_H__
#define __PIPELINE_PARAMETER_H__
#include <string>
#include <vector>
#include "camera_info/neu_camera_info.h"

typedef struct custom_timestamp
{
    unsigned int cam_num, sec, nanosec;
    void *task;
} TIME_STAMP;

enum class Perception2d
{
    ADD_TIMESTAMP,
    INJECTION_TIMESTAMP,
    PUBLISH_DETECTION_MESSAGE,
    PUBLISH_SEGMENTATION_MESSAGE
};

enum class CamNum
{
    CAM_F,
    CAM_FL,
    CAM_FR,
    CAM_BL,
    CAM_BR,
};

constexpr unsigned int FRAME_RATE = 15;
constexpr unsigned int MAX_CAM = 5;

class SuperParameter
{
public:
    SuperParameter();
    virtual ~SuperParameter();
    unsigned short get_video_width() const;
    unsigned short get_video_height() const;
    unsigned short get_input_width() const;
    unsigned short get_input_height() const;
    std::vector<std::string> get_cam_list() const;
    std::vector<unsigned short> get_crop_size() const;
    bool get_check_shm() const;
    std::string get_task_name() const;
    std::string get_model_config_name() const;

    void set_video_width(const unsigned short width);
    void set_video_height(const unsigned short height);
    void set_input_width(const unsigned short width);
    void set_input_height(const unsigned short height);
    void set_cam_list(const std::vector<std::string> &cam_list);
    void set_crop_size(const std::vector<int16_t> &crop_size);
    void set_check_shm(const bool use_shm);
    void set_task_name(const std::string &task_name);
    void set_model_config_name(const std::string &model_config_name);

    virtual std::vector<std::string> get_cam_list2() const;
    virtual void set_model_config_name2(const std::string &model_config_name);
    virtual std::string get_model_config_name2() const;
    virtual void set_cam_list2(const std::vector<std::string> &cam_list);
    virtual std::string get_tracker_config_name() const;
    virtual void set_tracker_config_name(const std::string tracker_config_name);

protected:
    std::string m_task_name, m_model_config_name;
    std::vector<unsigned short> m_crop_size;
    std::vector<std::string> m_cam_list;
    bool m_use_shm;
    unsigned short m_video_width, m_video_height, m_input_width, m_input_height;
};

class SegmentationParameter : public SuperParameter
{
public:
    SegmentationParameter();
    ~SegmentationParameter();
};

class DetectionParameter : public SuperParameter
{
public:
    DetectionParameter();
    ~DetectionParameter();

    void set_tracker_config_name(const std::string &tracker_config_name);
    std::string get_tracker_config_name() const;

private:
    std::string m_tracker_config_name;
};

#endif //__PIPELINE_PARAMETER_H__
