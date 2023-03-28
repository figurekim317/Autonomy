#include "image_dimension_transfomration/detection_transformer.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DetectionTransformer>("detection_transformer");

    rclcpp::spin(node);

    return 0;
}
