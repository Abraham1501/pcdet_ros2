#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

using namespace std::chrono_literals;


class PCDPublisher : public rclcpp::Node
{
public:
  PCDPublisher() : Node("pcd_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);

    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "/home/kfcnef/pcdet_ros2/pcdet_ros2/src/pcd_publisher/tec_openhouse_proccesed/";
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader_->open(storage_options, converter_options);

    // Get topic types
    auto topics = reader_->get_all_topics_and_types();
    for (const auto & topic : topics) {
        RCLCPP_INFO(this->get_logger(), "Found topic: %s [%s]", topic.name.c_str(), topic.type.c_str());
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    //if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/kfcnef/pcdet_ros2/pcdet_ros2/src/pcd_publisher/pcd/tec_map.pcd", cloud) == -1) {
    //  RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file");
    //  return;
    //}

    pcl::toROSMsg(cloud, output_);
    output_.header.frame_id = "map";

    timer_ = this->create_wall_timer(5ms, std::bind(&PCDPublisher::PublishMessage, this));
  }

private:
  void PublishMessage()
  {
    if (reader_->has_next()) {
      auto msg = reader_->read_next();
      if (msg->topic_name == "/points_rotated") { // Replace with your actual topic name
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        serializer.deserialize_message(&serialized_msg, &cloud_msg);
        publisher_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published point cloud at timestamp %d", cloud_msg.header.stamp.sec);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No more messages in bag. Shutting down.");
      rclcpp::shutdown();
    }
  }

  std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 output_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto pcd_publisher_node = std::make_shared<PCDPublisher>();
  rclcpp::spin(pcd_publisher_node);
  rclcpp::shutdown();
  return 0;
}
