cmake_minimum_required(VERSION 3.5)
project(pcd_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

#rosbag packages
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)


add_executable(pcd_publisher_node src/pcd_publisher_node.cpp)
ament_target_dependencies(pcd_publisher_node
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
  rosbag2_storage
)
target_link_libraries(pcd_publisher_node ${PCL_LIBRARIES})


add_executable(pcd_republisher_node src/pcd_republisher_node.cpp)
ament_target_dependencies(pcd_republisher_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pcd_republisher_node ${PCL_LIBRARIES})


add_executable(3Ddetections_to_markers src/3Ddetections_to_markers.cpp)
ament_target_dependencies(3Ddetections_to_markers
  rclcpp
  vision_msgs
  visualization_msgs
)
target_link_libraries(3Ddetections_to_markers ${PCL_LIBRARIES})


# Install targets
install(TARGETS
  pcd_publisher_node
  pcd_republisher_node
  3Ddetections_to_markers
  DESTINATION lib/${PROJECT_NAME})

ament_package()
