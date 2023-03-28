#include "panic_mode/panic_mode_detector.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PanicModeDetectorNode>("panic_mode_detector");

    rclcpp::spin(node);

    return 0;
}
