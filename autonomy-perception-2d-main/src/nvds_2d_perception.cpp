#include "neu_2d_perception.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("perception_2d");
    std::shared_ptr<NEU2Dperception> detection_node = std::make_shared<NEU2Dperception>(*node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
