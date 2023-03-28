#ifndef __PERCEPTION_2D_PARAMETER_H__
#define __PERCEPTION_2D_PARAMETER_H__

#include "pipeline/pipeline_parameter.h"

class Perception2dParameter : public SuperParameter
{
public:
    Perception2dParameter();
    virtual ~Perception2dParameter();

    virtual void set_model_config_name2(const std::string &model_config_name);
    virtual std::vector<std::string> get_cam_list2() const;
    virtual std::string get_model_config_name2() const;
    virtual void set_cam_list2(const std::vector<std::string> &cam_list);
    virtual std::string get_tracker_config_name() const;
    virtual void set_tracker_config_name(const std::string tracker_config_name);

private:
    std::string m_model_config_name2, m_tracker_config_name;
    std::vector<std::string> m_cam_list2;
};

#endif //__PERCEPTION_2D_PARAMETER_H__
