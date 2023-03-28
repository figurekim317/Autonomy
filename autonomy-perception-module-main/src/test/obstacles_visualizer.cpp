#include "test/obstacles_visualizer.h"
#include "camera_parameter_manager.h"
#include "perception2d_class_manager.h"

ObstaclesVisualizerNode::ObstaclesVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub) : m_node(node), m_pub(pub)
{
    run_node();
}

ObstaclesVisualizerNode::~ObstaclesVisualizerNode()
{
}

void ObstaclesVisualizerNode::run_node()
{
    std::cout << "obstacles visualizer process start" << std::endl;

    this->declare_ros_parameter();
    this->set_camera_parameter("f");

    this->set_custom_subscriber(m_custom_subscriber);

    std::function<void()> thread_function = std::bind(&ObstaclesVisualizerNode::visualizer_thread, this);
    m_thread = std::thread(thread_function);

    m_thread.detach();
}

void ObstaclesVisualizerNode::declare_ros_parameter()
{
    m_custom_topic_to_subscribe = m_node.declare_parameter("custom_topic_to_subscribe", "/seg_point");
}

void ObstaclesVisualizerNode::set_camera_parameter(const std::string cam_name)
{
    cv::Mat intrinsic_matrix_cv = CameraParameterReceiver::instance().get_cam_map()[cam_name].get_intrinsic_matrix();
    cv::Mat distortion_matrix_cv = CameraParameterReceiver::instance().get_cam_map()[cam_name].get_distortion_matrix();

    cv::Mat distortion_correction_matrix_cv = cv::getOptimalNewCameraMatrix(intrinsic_matrix_cv, distortion_matrix_cv, cv::Size(WIDTH, HEIGHT), 0.0, cv::Size(WIDTH, HEIGHT), 0);

    m_distortion_correction_matrix << distortion_correction_matrix_cv.at<double>(0, 0), distortion_correction_matrix_cv.at<double>(0, 1), distortion_correction_matrix_cv.at<double>(0, 2),
        distortion_correction_matrix_cv.at<double>(1, 0), distortion_correction_matrix_cv.at<double>(1, 1), distortion_correction_matrix_cv.at<double>(1, 2),
        distortion_correction_matrix_cv.at<double>(2, 0), distortion_correction_matrix_cv.at<double>(2, 1), distortion_correction_matrix_cv.at<double>(2, 2);
}

Eigen::Matrix3d ObstaclesVisualizerNode::get_camera_matrix() const
{
    return m_distortion_correction_matrix;
}

void ObstaclesVisualizerNode::set_custom_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::StereoObject>::SharedPtr &subscriber)
{
    std::function<void(std::shared_ptr<autonomy_ros2_message::msg::StereoObject>)> fnc = std::bind(&ObstaclesVisualizerNode::obstacles_callback, this, std::placeholders::_1);
    subscriber = m_node.create_subscription<autonomy_ros2_message::msg::StereoObject>(m_custom_topic_to_subscribe, rclcpp::SensorDataQoS(), fnc);
}

void ObstaclesVisualizerNode::obstacles_callback(const autonomy_ros2_message::msg::StereoObject::SharedPtr input)
{
    std::cout << "obstacles receive" << std::endl;

    m_custom_mutex.lock();

    m_point_cloud_data = *input;

    m_init_custom = true;

    m_custom_mutex.unlock();
}

void ObstaclesVisualizerNode::draw_bbox_rectangle(const double max_x, const double max_y, const double min_x, const double min_y, cv::Mat &src_img)
{
    cv::Point bb_pt = cv::Point(((int)(min_x * 1)), (int)(min_y * 1));
    cv::rectangle(src_img, cv::Rect(bb_pt, cv::Size((int)(max_x - min_x), (int)(max_y - min_y))), cv::Scalar(0, 0, 255), 3, 8, 0);
}

void ObstaclesVisualizerNode::project_to_image(const autonomy_ros2_message::msg::StereoObject::SharedPtr obs, cv::Mat &pc_img)
{
    for (size_t num_a = 0; num_a < obs->object.size(); ++num_a)
    {
        double pmax_x = -100000;
        double pmax_y = -100000;
        double pmin_x = 100000;
        double pmin_y = 100000;

        if (distance(obs->object[num_a].center) > 6)
        {
            continue;
        }

        for (size_t num_b = 0; num_b < obs->object[num_a].point.size(); ++num_b)
        {
            pcl::PointXYZI pt;

            // double x = obs->object[num_a].point[num_b].x;
            // double y = obs->object[num_a].point[num_b].y;
            // double z = obs->object[num_a].point[num_b].z;
            //////////////////////////////////////////////
            pt.x = obs->object[num_a].point[num_b].x;
            pt.y = obs->object[num_a].point[num_b].y;
            pt.z = obs->object[num_a].point[num_b].z;

            pt = translation(-0.23, -0.0, -0.41, pt); // 수정 반드시 필요!!!!!!!!!!! parameter-manager에도 반드시 depth extrinsic 추가 필요!!!!!!!!!

            double x = pt.x;
            double y = pt.y;
            double z = pt.z;
            ///////////////////////////////////////////////////////////

            Eigen::Vector3d in(-y, -z, x);
            Eigen::Vector3d out = m_distortion_correction_matrix * in;

            double u = out[0] / x;
            double v = out[1] / x;

            if (u < 0 || u >= WIDTH)
            {
                continue;
            }

            if (v < 0 || v >= HEIGHT)
            {
                continue;
            }

            if (pmax_x < u)
            {
                pmax_x = u;
            }

            if (pmin_x > u)
            {
                pmin_x = u;
            }

            if (pmax_y < v)
            {
                pmax_y = v;
            }

            if (pmin_y > v)
            {
                pmin_y = v;
            }
        }

        if (pmax_x == -100000 && pmax_y == -100000)
        {
            continue;
        }

        draw_bbox_rectangle(pmax_x, pmax_y, pmin_x, pmin_y, pc_img);
    }
}

void ObstaclesVisualizerNode::visualizer_thread()
{
    while (rclcpp::ok())
    {
        bool init_custom = m_init_custom;

        if (!init_custom)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        m_custom_mutex.lock();

        autonomy_ros2_message::msg::StereoObject::SharedPtr point_cloud_data(new autonomy_ros2_message::msg::StereoObject);
        *point_cloud_data = m_point_cloud_data;
        m_init_custom = false;

        m_custom_mutex.unlock();

        cv::Mat output_img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

        project_to_image(point_cloud_data, output_img);

        MatStamped out_custom_mat;

        out_custom_mat.matrix = output_img;
        out_custom_mat.stamp = point_cloud_data->header.stamp;

        m_pub->get_obstacles(out_custom_mat);
    }
}
