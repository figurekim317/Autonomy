#include "test/publisher.h"
#include "camera_parameter_manager.h"
#include "perception2d_class_manager.h"

PublisherNode::PublisherNode(rclcpp::Node &node) : m_node(node)
{
    run_node();
}

PublisherNode::~PublisherNode()
{
}

void PublisherNode::run_node()
{
    std::cout << "process start" << std::endl;

    this->declare_ros_parameter();
    this->set_result_publisher(m_msg_publisher);

    std::function<void()> thread_function = std::bind(&PublisherNode::visualizer_thread, this);
    m_thread = std::thread(thread_function);

    m_thread.detach();
}

void PublisherNode::declare_ros_parameter()
{
    m_msg_topic_to_publish = m_node.declare_parameter("msg_topic_to_publish", "/perception/visualizer_msg");
    m_frame_id_name = m_node.declare_parameter("frame_id_name", "base_link");
}

void PublisherNode::set_result_publisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher)
{
    publisher = m_node.create_publisher<sensor_msgs::msg::Image>(m_msg_topic_to_publish, QUEUE_SIZE);
}

void PublisherNode::get_obstacles(MatStamped input)
{
    m_custom_mutex.lock();

    m_point_cloud_data_vec.insert(m_point_cloud_data_vec.begin() + 0, input);
    if (m_point_cloud_data_vec.size() > STACK_SIZE)
    {
        m_point_cloud_data_vec.pop_back();
    }

    m_init_custom = true;

    m_custom_mutex.unlock();
}

void PublisherNode::get_detection(MatStamped input)
{
    m_detection_mutex.lock();

    m_detection_data_vec.insert(m_detection_data_vec.begin() + 0, input);
    if (m_detection_data_vec.size() > STACK_SIZE)
    {
        m_detection_data_vec.pop_back();
    }

    m_init_detection = true;

    m_detection_mutex.unlock();
}

void PublisherNode::get_segmentation(MatStamped input)
{
    m_seg_mutex.lock();

    m_seg_data_vec.insert(m_seg_data_vec.begin() + 0, input);
    if (m_seg_data_vec.size() > STACK_SIZE)
    {
        m_seg_data_vec.pop_back();
    }

    m_init_seg = true;

    m_seg_mutex.unlock();
}

void PublisherNode::get_img(MatStamped input)
{
    m_img_mutex.lock();

    m_img_data_vec.insert(m_img_data_vec.begin() + 0, input);
    if (m_img_data_vec.size() > STACK_SIZE)
    {
        m_img_data_vec.pop_back();
    }

    m_init_img = true;

    m_img_mutex.unlock();
}

void PublisherNode::synchronize_data(const std::vector<MatStamped> src_vec, const std::vector<MatStamped> dst_vec,
                                     MatStamped &src, MatStamped &dst, bool &sync)
{
    bool find_it = false;

    try
    {
        for (size_t num_i = 0; num_i < src_vec.size(); num_i++)
        {
            for (size_t num_j = 0; num_j < dst_vec.size(); num_j++)
            {
                if (sync)
                {
                    if (fabs(double(src.stamp.seconds()) + double(src.stamp.nanoseconds()) / 1000000000.0 -
                             double(dst_vec[num_j].stamp.seconds()) - double(dst_vec[num_j].stamp.nanoseconds()) / 1000000000.0) < FPS_FRATION * 2 / 3)
                    {
                        dst = dst_vec[num_j];

                        find_it = true;

                        throw find_it;
                    }
                    else if (fabs(double(src.stamp.seconds()) + double(src.stamp.nanoseconds()) / 1000000000.0 -
                                  double(dst_vec[num_j].stamp.seconds()) - double(dst_vec[num_j].stamp.nanoseconds()) / 1000000000.0) < FPS_FRATION * STACK_SIZE)
                    {
                        dst = dst_vec[0];
                    }
                }
                else if (fabs(double(src_vec[num_i].stamp.seconds()) + double(src_vec[num_i].stamp.nanoseconds()) / 1000000000.0 -
                              double(dst_vec[num_j].stamp.seconds()) - double(dst_vec[num_j].stamp.nanoseconds()) / 1000000000.0) < FPS_FRATION / 2)
                {
                    src = src_vec[num_i];
                    dst = dst_vec[num_j];

                    find_it = true;

                    throw find_it;
                }
            }
        }
    }
    catch (bool &e)
    {
        sync = true;
        std::cout << "good" << std::endl;
    }
}

// void PublisherNode::show_result(MatStamped src_img, std::string window_name, int widow_size_width, int widow_size_height)
// {
//     cv::namedWindow(window_name, cv::WINDOW_NORMAL);

//     cv::resizeWindow(window_name, widow_size_width, widow_size_height);

//     cv::imshow(window_name, src_img);

//     cv::waitKey(30);
// }

void PublisherNode::visualizer_thread()
{
    while (rclcpp::ok())
    {
        bool init_img = m_init_img;

        if (!init_img || m_img_data_vec.size() < 4)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        m_img_mutex.lock();

        std::vector<MatStamped> img_data_vec = m_img_data_vec;
        m_init_img = false;

        m_img_mutex.unlock();

        std::vector<MatStamped> detection_data_vec;

        m_detection_mutex.lock();

        detection_data_vec = m_detection_data_vec;
        m_init_detection = false;

        m_detection_mutex.unlock();

        std::vector<MatStamped> point_cloud_data_vec;

        m_custom_mutex.lock();

        point_cloud_data_vec = m_point_cloud_data_vec;
        m_init_custom = false;

        m_custom_mutex.unlock();

        std::vector<MatStamped> seg_data_vec;

        m_seg_mutex.lock();

        seg_data_vec = m_seg_data_vec;
        m_init_seg = false;

        m_seg_mutex.unlock();

        MatStamped img_data;
        MatStamped detection_data;
        MatStamped obstacles_data;
        MatStamped seg_data;

        bool is_sync = false;

        synchronize_data(img_data_vec, detection_data_vec, img_data, detection_data, is_sync);
        synchronize_data(img_data_vec, point_cloud_data_vec, img_data, obstacles_data, is_sync);
        synchronize_data(img_data_vec, seg_data_vec, img_data, seg_data, is_sync);

        cv::Mat output_img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

        img_data.matrix.copyTo(output_img);

        if (!is_sync)
        {
            output_img = img_data_vec[0].matrix;
        }

        if (seg_data.stamp.seconds() != 0)
        {
            for (int num_x = 0; num_x < seg_data.matrix.cols; num_x++)
            {
                for (int num_y = 0; num_y < seg_data.matrix.rows; num_y++)
                {
                    if (seg_data.matrix.data[(num_y * seg_data.matrix.cols + num_x) * 3] != 0 ||
                        seg_data.matrix.data[(num_y * seg_data.matrix.cols + num_x) * 3 + 1] != 0 ||
                        seg_data.matrix.data[(num_y * seg_data.matrix.cols + num_x) * 3 + 2] != 0)
                    {
                        int origin_bgr_b = output_img.data[((num_y + CROP) * seg_data.matrix.cols + num_x) * 3];
                        int origin_bgr_g = output_img.data[((num_y + CROP) * seg_data.matrix.cols + num_x) * 3 + 1];
                        int origin_bgr_r = output_img.data[((num_y + CROP) * seg_data.matrix.cols + num_x) * 3 + 2];
                        int bgr_b = origin_bgr_b + seg_data.matrix.data[(num_y * seg_data.matrix.cols + num_x) * 3];
                        int bgr_g = origin_bgr_g + seg_data.matrix.data[(num_y * seg_data.matrix.cols + num_x) * 3 + 1];
                        int bgr_r = origin_bgr_r + seg_data.matrix.data[(num_y * seg_data.matrix.cols + num_x) * 3 + 2];
                        output_img.data[((num_y + CROP) * seg_data.matrix.cols + num_x) * 3] = bgr_b;
                        output_img.data[((num_y + CROP) * seg_data.matrix.cols + num_x) * 3 + 1] = bgr_g;
                        output_img.data[((num_y + CROP) * seg_data.matrix.cols + num_x) * 3 + 2] = bgr_r;
                    }
                }
            }
            // std::cout << "seg_data.matrix.rows : " << seg_data.matrix.rows << std::endl;
        }

        if (obstacles_data.stamp.seconds() != 0)
        {
            for (int num_x = 0; num_x < output_img.cols; num_x++)
            {
                for (int num_y = 0; num_y < output_img.rows; num_y++)
                {
                    if (obstacles_data.matrix.data[(num_y * output_img.cols + num_x) * 3] != 0 ||
                        obstacles_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 1] != 0 ||
                        obstacles_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 2] != 0)
                    {
                        output_img.data[(num_y * output_img.cols + num_x) * 3] = obstacles_data.matrix.data[(num_y * output_img.cols + num_x) * 3];
                        output_img.data[(num_y * output_img.cols + num_x) * 3 + 1] = obstacles_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 1];
                        output_img.data[(num_y * output_img.cols + num_x) * 3 + 2] = obstacles_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 2];
                    }
                }
            }
        }

        if (detection_data.stamp.seconds() != 0)
        {
            for (int num_x = 0; num_x < output_img.cols; num_x++)
            {
                for (int num_y = 0; num_y < output_img.rows; num_y++)
                {
                    if (detection_data.matrix.data[(num_y * output_img.cols + num_x) * 3] != 0 ||
                        detection_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 1] != 0 ||
                        detection_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 2] != 0)
                    {
                        output_img.data[(num_y * output_img.cols + num_x) * 3] = detection_data.matrix.data[(num_y * output_img.cols + num_x) * 3];
                        output_img.data[(num_y * output_img.cols + num_x) * 3 + 1] = detection_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 1];
                        output_img.data[(num_y * output_img.cols + num_x) * 3 + 2] = detection_data.matrix.data[(num_y * output_img.cols + num_x) * 3 + 2];
                    }
                }
            }
        }

        sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image);
        std_msgs::msg::Header output_header;
        output_header.stamp = img_data_vec[0].stamp;
        msg = cv_bridge::CvImage(output_header, "bgr8", output_img).toImageMsg();

        m_msg_publisher->publish(*msg);
    }
}
