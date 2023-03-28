#include "test/segmentation_visualizer.h"
#include "camera_parameter_manager.h"
#include "perception2d_class_manager.h"

SegmentationVisualizerNode::SegmentationVisualizerNode(rclcpp::Node &node, std::shared_ptr<PublisherNode> pub) : m_node(node), m_pub(pub)
{
    run_node();
}

SegmentationVisualizerNode::~SegmentationVisualizerNode()
{
}

void SegmentationVisualizerNode::run_node()
{
    std::cout << "segmentation visualizer process start" << std::endl;

    this->declare_ros_parameter();

    this->set_segmentation_subscriber(m_segmentation_subscriber);

    std::function<void()> thread_function = std::bind(&SegmentationVisualizerNode::visualizer_thread, this);
    m_thread = std::thread(thread_function);

    m_thread.detach();
}

void SegmentationVisualizerNode::declare_ros_parameter()
{
    m_segmentation_topic_to_subscribe = m_node.declare_parameter("segmentation_topic_to_subscribe", "/cam_f_semantic_segmentation");
}

void SegmentationVisualizerNode::set_segmentation_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &subscriber)
{
    std::function<void(std::shared_ptr<autonomy_ros2_message::msg::NEUOccupancyGrid>)> fnc = std::bind(&SegmentationVisualizerNode::segmentation_callback, this, std::placeholders::_1);
    subscriber = m_node.create_subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>(m_segmentation_topic_to_subscribe, rclcpp::SensorDataQoS(), fnc);
}

void SegmentationVisualizerNode::segmentation_callback(const autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr input)
{
    std::cout << "segmentation receive" << std::endl;

    m_segmentation_mutex.lock();

    m_segmentation_data = *input;

    m_init_segmentation = true;

    m_segmentation_mutex.unlock();
}

void SegmentationVisualizerNode::visualizer_thread()
{
    while (rclcpp::ok())
    {
        bool init_segmentation = m_init_segmentation;

        if (!init_segmentation)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        m_segmentation_mutex.lock();

        autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr segmentation_data(new autonomy_ros2_message::msg::NEUOccupancyGrid);
        *segmentation_data = m_segmentation_data;
        m_init_segmentation = false;

        m_segmentation_mutex.unlock();

        cv::Mat seg_class(SEG_HEIGHT, SEG_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

        for (size_t idx_i = 0; idx_i < segmentation_data->classes.size(); idx_i++)
        {
            if (segmentation_data->classes[idx_i] != 0)
            {
                seg_class.data[idx_i * 3] = 255 * segmentation_data->classes[idx_i] % 255;
                seg_class.data[idx_i * 3 + 1] = 50 * segmentation_data->classes[idx_i] % 255;
                seg_class.data[idx_i * 3 + 2] = 125 * segmentation_data->classes[idx_i] % 255;
            }
        }

        cv::Mat seg_class_large;
        cv::resize(seg_class, seg_class_large, cv::Size(WIDTH, HEIGHT - CROP), 0, 0, 0);

        cv::Mat output_img;
        output_img = seg_class_large;

        MatStamped out_custom_mat;

        out_custom_mat.matrix = output_img;
        out_custom_mat.stamp = segmentation_data->header.stamp;

        m_pub->get_segmentation(out_custom_mat);
    }
}
