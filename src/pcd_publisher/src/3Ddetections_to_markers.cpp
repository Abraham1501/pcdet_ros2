#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;

class DetectionVisualizer : public rclcpp::Node {
public:
    DetectionVisualizer()
        : Node("detection3d_visualizer")
    {
        sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/cloud_detections", 10,
            std::bind(&DetectionVisualizer::callback, this, _1)
        );

        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detection_markers", 10
        );

        RCLCPP_INFO(this->get_logger(), "Detection Visualizer Node started");
    }

private:
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

    void callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto& detection : msg->detections) {
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "detections";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose = detection.bbox.center;
            marker.scale = detection.bbox.size;

            // Color: semi-transparent green
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);  // optional

            marker_array.markers.push_back(marker);
        }

        pub_->publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionVisualizer>());
    rclcpp::shutdown();
    return 0;
}
