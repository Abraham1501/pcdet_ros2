#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;


class PointCloudRepublisher : public rclcpp::Node
{
public:
  PointCloudRepublisher() : Node("PointCloudRepublisher")
  {
    subscribe_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  "kitti/point_cloud", 10,
  std::bind(&PointCloudRepublisher::callback, this, std::placeholders::_1));


    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);


    // Get topic types
    //auto topics_reader = Node->get_all_topics_and_types();
    //for (const auto & topic : topics_reader) {
    //    RCLCPP_INFO(this->get_logger(), "Found topic: %s [%s]", topic.name.c_str(), topic.type.c_str());
    //}

    RCLCPP_INFO(this->get_logger(), "PointCloud republisher started.");

  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Republish the message as-is
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto pcd_republisher_node = std::make_shared<PointCloudRepublisher>();
  rclcpp::spin(pcd_republisher_node);
  rclcpp::shutdown();
  return 0;
}
