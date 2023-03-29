#include "neu_detection.h"

void setting(std::shared_ptr<NEUPerception2D> test)
{
    for (int i = 0; i < MAX_CAM; ++i)
    {
        if (test->get_parameter(i) != nullptr)
        {
            std::vector<std::string> cam_list = test->get_parameter(i)->get_cam_list();
            std::cout << "-- cam list --" << std::endl;
            for (int i = 0; i < cam_list.size(); ++i)
            {
                std::cout << cam_list[i] << std::endl;
            }
            std::cout << "---------------" << std::endl;
            unsigned int idx = 0;
            for (int i = 0; i < cam_list.size(); ++i)
            {
                idx = test->get_perception(i)->get_cam_num(cam_list[i]);
                test->get_subs(idx)->set_subscriber(test->get_node_());
            }
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("object_detection");
    std::shared_ptr<NEUDetection> detection_node = std::make_shared<NEUDetection>(*node);
    setting(detection_node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
