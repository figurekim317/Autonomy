#include "test/image_visualizer.h"
#include "camera_parameter_manager.h"
#include "perception2d_class_manager.h"

ImageVisualizerNode::ImageVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub) : m_node(node), m_pub(pub)
{
    run_node();
}

ImageVisualizerNode::~ImageVisualizerNode()
{
}

void ImageVisualizerNode::run_node()
{
    std::cout << "image visualizer process start" << std::endl;

    this->declare_ros_parameter();

    this->set_img_subscriber(m_img_subscriber);

    std::function<void()> thread_function = std::bind(&ImageVisualizerNode::visualizer_thread, this);
    m_thread = std::thread(thread_function);

    m_thread.detach();
}

void ImageVisualizerNode::declare_ros_parameter()
{
    m_img_topic_to_subscribe = m_node.declare_parameter("img_topic_to_subscribe", "/cam_f/image/compressed");
}

void ImageVisualizerNode::set_img_subscriber(rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr &subscriber)
{
    std::function<void(std::shared_ptr<sensor_msgs::msg::CompressedImage>)> fnc = std::bind(&ImageVisualizerNode::img_callback, this, std::placeholders::_1);
    subscriber = m_node.create_subscription<sensor_msgs::msg::CompressedImage>(m_img_topic_to_subscribe, rclcpp::SensorDataQoS(), fnc);
}

void ImageVisualizerNode::img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr &input)
{
    std::cout << "img receive" << std::endl;

    m_img_mutex.lock();

    m_img_data = *input;
    m_init_img = true;

    m_img_mutex.unlock();
}

void ImageVisualizerNode::convert_compressed_to_cv(const sensor_msgs::msg::CompressedImage::SharedPtr src, cv::Mat &dst)
{
    try
    {
        if (!src->data.empty())
        {
            cv_bridge::CvImagePtr temp = cv_bridge::toCvCopy(*src, "bgr8");
            temp->image.copyTo(dst);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(m_node.get_logger(), "Could not convert to image!");
    }
}

void ImageVisualizerNode::visualizer_thread()
{
    while (rclcpp::ok())
    {
        bool init_img = m_init_img;

        if (!init_img)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        m_img_mutex.lock();

        sensor_msgs::msg::CompressedImage::SharedPtr img_data(new sensor_msgs::msg::CompressedImage);
        *img_data = m_img_data;
        m_init_img = false;

        m_img_mutex.unlock();

        cv::Mat output_img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

        convert_compressed_to_cv(img_data, output_img);

        MatStamped out_custom_mat;

        out_custom_mat.matrix = output_img;
        out_custom_mat.stamp = img_data->header.stamp;

        m_pub->get_img(out_custom_mat);
    }
}
