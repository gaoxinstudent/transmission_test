cmake_minimum_required(VERSION 3.5)
project(video_streaming_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 添加OpenCV的依赖
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} # 添加OpenCV的头文件目录
)

add_executable(video_publisher_node src/video_publisher_node.cpp)
target_link_libraries(video_publisher_node yaml-cpp)

ament_target_dependencies(video_publisher_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(video_subscriber_node src/video_subscriber_node.cpp)
target_link_libraries(video_subscriber_node yaml-cpp)

ament_target_dependencies(video_subscriber_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  video_publisher_node
  video_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
