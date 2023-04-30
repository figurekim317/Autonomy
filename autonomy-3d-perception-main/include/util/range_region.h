

fndef __RANGE_REGION_H__
#define __RANGE_REGION_H__

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

constexpr double ANGLE = 170;

class RangeRegion : public rclcpp::Node
{
public:
    RangeRegion(std::string node_name);

    void timer_callback();

private:
    visualization_msgs::msg::Marker init_line_strip(const std::string &frame_id, const int id);
    void draw_line(const double &max_range, const double &min_range, visualization_msgs::msg::Marker &line);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_line_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
};

#endif // __RANGE_REGION_H__
