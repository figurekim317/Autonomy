#include "panic_mode/panic_mode_detector.h"

PanicModeDetectorNode::PanicModeDetectorNode(const std::string node_name) : Node(node_name)
{
    run_node();
}

PanicModeDetectorNode::~PanicModeDetectorNode()
{
}

void PanicModeDetectorNode::run_node()
{
    std::cout << "process start" << std::endl;

    this->declare_ros_parameter();

    this->set_seg_subscriber(m_seg_subscriber);
    this->set_result_publisher(m_msg_publisher);

    std::function<void()> thread_function = bind(&PanicModeDetectorNode::detector_thread, this);
    m_thread = std::thread(thread_function);

    m_thread.detach();
}

void PanicModeDetectorNode::declare_ros_parameter()
{
    m_seg_topic_to_subscribe = this->declare_parameter("seg_topic_to_subscribe", "/road_segmentation_image");
    m_msg_topic_to_publish = this->declare_parameter("msg_topic_to_publish", "/drivable_area_count");
}

void PanicModeDetectorNode::set_seg_subscriber(rclcpp::Subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>::SharedPtr &subscriber)
{
    std::function<void(std::shared_ptr<autonomy_ros2_message::msg::NEUOccupancyGrid>)> fnc = std::bind(&PanicModeDetectorNode::seg_callback, this, std::placeholders::_1);
    subscriber = this->create_subscription<autonomy_ros2_message::msg::NEUOccupancyGrid>(m_seg_topic_to_subscribe, rclcpp::SensorDataQoS(), fnc);
}

void PanicModeDetectorNode::set_result_publisher(rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr &publisher)
{
    publisher = this->create_publisher<std_msgs::msg::Int64>(m_msg_topic_to_publish, QUEUE_SIZE);
}

void PanicModeDetectorNode::seg_callback(const autonomy_ros2_message::msg::NEUOccupancyGrid::SharedPtr input)
{
    m_segmentation_mutex.lock();

    m_segmentation_data = *input;

    m_init_segmentation = true;

    m_segmentation_mutex.unlock();
}

void PanicModeDetectorNode::pub_status_msg(const int drivable_area_count)
{
    std_msgs::msg::Int64::SharedPtr output(new std_msgs::msg::Int64);
    output->data = drivable_area_count;
    m_msg_publisher->publish(*output);
}

void PanicModeDetectorNode::detector_thread()
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

        autonomy_ros2_message::msg::NEUOccupancyGrid segmentation_data = m_segmentation_data;
        m_init_segmentation = false;

        m_segmentation_mutex.unlock();

        cv::Mat seg_class(90, 160, CV_8UC1, cv::Scalar(0));

        for (size_t idx_i = 0; idx_i < segmentation_data.classes.size(); idx_i++)
        {
            if (segmentation_data.classes[idx_i] != 0)
            {
                seg_class.data[idx_i] = segmentation_data.classes[idx_i];
            }
        }

        cv::Rect roi_panic(40, 60, 80, 30);

        int drivable_area_count = cv::countNonZero(seg_class(roi_panic));

        pub_status_msg(drivable_area_count);
    }
}
