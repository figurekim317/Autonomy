#include "test/bbox_visualizer.h"
#include "camera_parameter_manager.h"
#include "perception2d_class_manager.h"

BboxVisualizerNode::BboxVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub) : m_node(node), m_pub(pub)
{
    run_node();
}

BboxVisualizerNode::~BboxVisualizerNode()
{
}

void BboxVisualizerNode::run_node()
{
    std::cout << "bbox visualizer process start" << std::endl;

    this->declare_ros_parameter();

    this->set_detection_subscriber(m_detection_subscriber);

    std::function<void()> thread_function = std::bind(&BboxVisualizerNode::visualizer_thread, this);
    m_thread = std::thread(thread_function);

    m_thread.detach();
}

void BboxVisualizerNode::declare_ros_parameter()
{
    m_detection_topic_to_subscribe = m_node.declare_parameter("detection_topic_to_subscribe", "/object_detection");
}

void BboxVisualizerNode::set_detection_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUDetection>::SharedPtr &subscriber)
{
    std::function<void(std::shared_ptr<autonomy_ros2_message::msg::NEUDetection>)> fnc = std::bind(&BboxVisualizerNode::detection_callback, this, std::placeholders::_1);
    subscriber = m_node.create_subscription<autonomy_ros2_message::msg::NEUDetection>(m_detection_topic_to_subscribe, rclcpp::SensorDataQoS(), fnc);
}

void BboxVisualizerNode::detection_callback(const autonomy_ros2_message::msg::NEUDetection::SharedPtr &input)
{
    std::cout << "bbox receive" << std::endl;

    m_detection_mutex.lock();

    m_detection_data = *input;
    m_init_detection = true;

    m_detection_mutex.unlock();
}

void BboxVisualizerNode::draw_bbox_rectangle(const autonomy_ros2_message::msg::NEUBbox neu_box, cv::Mat &src_img)
{
    cv::Point bb_pt = cv::Point((int)(neu_box.point.u * 1) - (int)(neu_box.size.width * 1 / 2), (int)(neu_box.point.v * 1) - (int)(neu_box.size.height * 1 / 2));
    cv::rectangle(src_img, cv::Rect(bb_pt, cv::Size((int)(neu_box.size.width * 1), (int)(neu_box.size.height * 1))), cv::Scalar((neu_box.classes.id * 50) % 255, (neu_box.classes.id * 125) % 255, (neu_box.classes.id * 212) % 255), 3, 8, 0);
}

void BboxVisualizerNode::draw_bbox_text(const autonomy_ros2_message::msg::NEUBbox neu_box, cv::Mat &src_img)
{
    std::string txt = Perception2dClassListReceiver::instance().get_object_detection_class()[neu_box.classes.id];

    cv::Point my_point;
    my_point.x = (int)(neu_box.point.u * 1) - (int)(neu_box.size.width * 1 / 2);
    my_point.y = (int)(neu_box.point.v * 1) - (int)(neu_box.size.height * 1 / 2);

    int my_font_face = 2;

    double my_font_scale = 1.2;

    cv::putText(src_img, txt, my_point, my_font_face, my_font_scale, cv::Scalar::all(255));
}

void BboxVisualizerNode::visualizer_thread()
{
    while (rclcpp::ok())
    {
        bool init_detection = m_init_detection;

        if (!init_detection)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        m_detection_mutex.lock();

        autonomy_ros2_message::msg::NEUDetection::SharedPtr detection_data(new autonomy_ros2_message::msg::NEUDetection);
        *detection_data = m_detection_data;
        m_init_detection = false;

        m_detection_mutex.unlock();

        cv::Mat output_img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

        for (size_t idx_i = 0; idx_i < detection_data->bboxes.size(); idx_i++)
        {
            draw_bbox_rectangle(detection_data->bboxes[idx_i], output_img);
            draw_bbox_text(detection_data->bboxes[idx_i], output_img);
        }

        MatStamped out_custom_mat;

        out_custom_mat.matrix = output_img;
        out_custom_mat.stamp = detection_data->header.stamp;

        m_pub->get_detection(out_custom_mat);
    }
}
