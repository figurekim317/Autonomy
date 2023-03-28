#include "test/publisher.h"
#include "test/image_visualizer.h"
#include "test/bbox_visualizer.h"
#include "test/obstacles_visualizer.h"
#include "test/segmentation_visualizer.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("perception_visualizer");

    std::shared_ptr<PublisherNode> publisher = std::make_shared<PublisherNode>(*node);

    std::shared_ptr<ImageVisualizerNode> image_visualizer = std::make_shared<ImageVisualizerNode>(*node, publisher);

    std::shared_ptr<BboxVisualizerNode> bbox_visualizer = std::make_shared<BboxVisualizerNode>(*node, publisher);

    // std::shared_ptr<ObstaclesVisualizerNode> obstacles_visualizer = std::make_shared<ObstaclesVisualizerNode>(*node, publisher);

    std::shared_ptr<SegmentationVisualizerNode> segmentation_visualizer = std::make_shared<SegmentationVisualizerNode>(*node, publisher);

    rclcpp::spin(node);

    return 0;
}
