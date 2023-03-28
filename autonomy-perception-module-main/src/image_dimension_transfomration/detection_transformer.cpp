#include "image_dimension_transfomration/detection_transformer.h"
#include "perception2d_class_manager.h"
#include <cmath>
#include <functional>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

DetectionTransformer::DetectionTransformer(const std::string &node_name)
    : rclcpp::Node(node_name),
      m_cam_names(this->declare_parameter("cam_names", std::vector<std::string>{"f", "br", "bl"})),
      m_except_class(this->declare_parameter("except_class", std::vector<std::string>{})),
      m_ground_class(this->declare_parameter("ground_class", std::vector<std::string>{"bicycle", "car", "motorcycle", "person", "scooter", "dog", "bollard", "cart"})),
      m_cam_res_height(this->declare_parameter("cam_res_height", 1944)),
      m_cam_res_width(this->declare_parameter("cam_res_width", 2592)),
      m_max_distance_threshold(this->declare_parameter("max_distance_threshold", 30.)),
      m_under_principle_point_distance_thr(this->declare_parameter("under_principle_point_distance_thr", 13.)),
      m_is_debug(this->declare_parameter("is_debug", false)),
      m_mode(this->declare_parameter("mode", 0)),
      m_last_check_z_value(this->declare_parameter("last_check_z_value", 5.)),
      m_check_z_linspace_value(this->declare_parameter("check_z_linspace_value", .5))
{
    for (std::string &cam_name : m_cam_names)
    {
        this->set_image_3d_point_publisher(cam_name);
        this->set_detection_subs_callback(cam_name);
    }

    for (auto m : Perception2dClassListReceiver::instance().get_object_detection_class())
    {
        this->declare_parameter(m.second + "_prior_height", 0.);
    }

    m_transformer = std::make_shared<Transformer>(m_cam_names);
}

DetectionTransformer::~DetectionTransformer() {}

void DetectionTransformer::set_detection_subs_callback(std::string &cam_name)
{
    std::function<void(std::shared_ptr<autonomy_ros2_message::msg::NEUDetection>)> fnc = std::bind(&DetectionTransformer::detection_sub_callback, this, cam_name, std::placeholders::_1);
    rclcpp::Subscription<autonomy_ros2_message::msg::NEUDetection>::SharedPtr subscriber = this->create_subscription<autonomy_ros2_message::msg::NEUDetection>("/cam_" + cam_name + "_object_detection", rclcpp::SensorDataQoS(), fnc);
    m_od_subscriber_map[cam_name] = subscriber;
}

void DetectionTransformer::set_image_3d_point_publisher(std::string &cam_name)
{
    rclcpp::Publisher<autonomy_ros2_message::msg::NEUDetection>::SharedPtr publisher;
    publisher = this->create_publisher<autonomy_ros2_message::msg::NEUDetection>("/cam_" + cam_name + "_image_3d_point", rclcpp::SensorDataQoS());
    m_od_publisher_map[cam_name] = publisher;
}

bool DetectionTransformer::is_except_class(const int class_id)
{
    this->get_parameter("except_class", m_except_class);

    if (m_except_class.empty())
    {
        return false;
    }
    else
    {
        return std::find(m_except_class.begin(), m_except_class.end(),
                         Perception2dClassListReceiver::instance().get_object_detection_class()[class_id]) != m_except_class.end();
    }
}

bool DetectionTransformer::is_ground_class(const int class_id)
{
    this->get_parameter("ground_class", m_ground_class);

    if (m_ground_class.empty())
    {
        return false;
    }
    else
    {
        return std::find(m_ground_class.begin(), m_ground_class.end(),
                         Perception2dClassListReceiver::instance().get_object_detection_class()[class_id]) != m_except_class.end();
    }
}

bool DetectionTransformer::is_proper_distance(const double distance)
{
    this->get_parameter("max_distance_threshold", m_max_distance_threshold);

    return distance <= m_max_distance_threshold;
}

void DetectionTransformer::detection_sub_callback(std::string &cam_name, autonomy_ros2_message::msg::NEUDetection::SharedPtr detection_msg)
{
    this->get_parameter("cam_height", m_cam_res_height);
    this->get_parameter("cam_width", m_cam_res_width);
    this->get_parameter("under_principle_point_distance_thr", m_under_principle_point_distance_thr);
    this->get_parameter("last_check_z_value", m_last_check_z_value);
    this->get_parameter("check_z_linspace_value", m_check_z_linspace_value);
    this->get_parameter("is_debug", m_is_debug);

    m_detection_msg.header = detection_msg->header;

    cv::Point2d bbox_in_image_bottom_center;
    cv::Point2d bbox_in_image_top_center;
    cv::Point3d bbox_in_world_bottom_center;
    cv::Point3d bbox_in_world_top_center;
    cv::Point3d bbox_in_world_result;
    cv::Point2d bsize_in_image;
    cv::Point2d bbox_top_reprojection;

    double distance;
    double scale_factor;
    double top_center_y_reprojection_error = 1000.;
    double mse_error;

    m_prior_height = 0.;
    for (autonomy_ros2_message::msg::NEUBbox &bbox : detection_msg->bboxes)
    {
        if (this->is_except_class(bbox.classes.id))
            continue;

        if (bbox.point.u < 0 || bbox.point.v < 0 || bbox.point.u > m_cam_res_width || bbox.point.v > m_cam_res_height)
            continue;

        bbox_in_image_bottom_center.x = bbox.point.u;
        bbox_in_image_bottom_center.y = bbox.point.v + (bbox.size.height / 2);

        bbox_in_image_top_center.x = bbox.point.u;
        bbox_in_image_top_center.y = bbox.point.v - (bbox.size.height / 2);

        if (bbox_in_image_bottom_center.y > m_cam_res_height)
        {
            bbox_in_image_bottom_center.y = m_cam_res_height;
            std::cout << "[Warning] bottom center y point greater than image height" << std::endl;
        }

        if (bbox_in_image_bottom_center.x > m_cam_res_width)
        {
            bbox_in_image_bottom_center.x = m_cam_res_width;
            std::cout << "[Warning] bottom center x point greater than image height" << std::endl;
        }

        if (bbox_in_image_top_center.y < 0)
        {
            bbox_in_image_top_center.y = 0;
            std::cout << "[Warning] top center y point less than 0" << std::endl;
        }

        if (bbox_in_image_top_center.x < 0)
        {
            bbox_in_image_top_center.x = 0;
            std::cout << "[Warning] top center x point less than 0" << std::endl;
        }

        bbox_in_world_result.x = 0;
        bbox_in_world_result.y = 0;
        bbox_in_world_result.z = 0;

        if (m_is_debug)
        {
            this->get_parameter("mode", m_mode);
        }
        else
        {
            if (m_transformer->is_over_principal_point(cam_name, bbox_in_image_bottom_center))
                m_mode = 1;
            else
            {
                if (this->is_ground_class(bbox.classes.id))
                    m_mode = 0;
                else
                    m_mode = 1;
            }
        }

        std::string class_name = Perception2dClassListReceiver::instance().get_object_detection_class()[bbox.classes.id];
        try
        {
            this->get_parameter(class_name + "_prior_height", m_prior_height);
        }
        catch (std::exception &e)
        {
            std::cout << e.what() << std::endl;
            m_prior_height = 0.;
        }

        switch (m_mode)
        {
        case 0:
            bbox_in_world_result = m_transformer->transform_cam_to_robot_using_3d_transformation(cam_name, bbox_in_image_bottom_center, 0);

            if ((std::sqrt(std::pow(bbox_in_world_result.x, 2) + std::pow(bbox_in_world_result.y, 2)) > m_under_principle_point_distance_thr) &&
                (m_prior_height != 0.) && (!(m_is_debug)))
            {
                cv::Point2d b_center = cv::Point2d(bbox.size.width, bbox.size.height);
                bbox_in_world_result = m_transformer->transform_cam_to_robot_using_prior_height(cam_name, bbox_in_image_bottom_center, b_center, m_prior_height);
            }
            break;

        case 1:
            if (m_prior_height != 0.)
            {
                cv::Point2d b_center = cv::Point2d(bbox.size.width, bbox.size.height);
                bbox_in_world_result = m_transformer->transform_cam_to_robot_using_prior_height(cam_name, bbox_in_image_bottom_center, b_center, m_prior_height);
            }
            break;

        case 2:
            if (m_prior_height != 0.)
            {
                for (double i = 0; i < m_last_check_z_value; i += m_check_z_linspace_value)
                {
                    bbox_in_world_bottom_center = m_transformer->transform_cam_to_robot_using_3d_transformation(cam_name, bbox_in_image_bottom_center, i);
                    bbox_in_world_top_center.x = bbox_in_world_bottom_center.x;
                    bbox_in_world_top_center.y = bbox_in_world_bottom_center.y;
                    bbox_in_world_top_center.z = bbox_in_world_bottom_center.z + m_prior_height;

                    scale_factor = m_transformer->get_current_scale_factor();
                    bbox_top_reprojection = m_transformer->transform_robot_to_cam_using_3d_transformation(cam_name, bbox_in_world_top_center, scale_factor);
                    mse_error = std::sqrt(std::pow(bbox_in_image_top_center.y - bbox_top_reprojection.y, 2));

                    if (top_center_y_reprojection_error > mse_error)
                    {
                        top_center_y_reprojection_error = mse_error;
                        bbox_in_world_result.x = bbox_in_world_top_center.x;
                        bbox_in_world_result.y = bbox_in_world_top_center.y;
                        bbox_in_world_result.z = bbox_in_world_bottom_center.z;
                    }
                }
            }
            break;

        default:
            std::cerr << "mode should be only 0, 1, 2" << std::endl;

            bbox_in_world_result.x = 0;
            bbox_in_world_result.y = 0;
            bbox_in_world_result.z = 0;
            break;
        }

        if (this->is_proper_distance(std::sqrt(std::pow(bbox_in_world_result.x, 2) + std::pow(bbox_in_world_result.y, 2))) && bbox_in_world_result.x != 0 && bbox_in_world_result.y != 0)
        {
            bbox.position.x = bbox_in_world_result.x;
            bbox.position.y = bbox_in_world_result.y;
            bbox.position.z = bbox_in_world_result.z;

            m_detection_msg.bboxes.emplace_back(bbox);
        }
    }

    if (!m_detection_msg.bboxes.empty())
    {
        m_od_publisher_map[cam_name]->publish(m_detection_msg);
        m_detection_msg.bboxes.clear();
    }
}
