#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

class VideoSubscriber : public rclcpp::Node
{
public:
    VideoSubscriber(const std::string &config_file)
    : Node("video_subscriber")
    {
        // 读取 YAML 配置文件
        YAML::Node config = YAML::LoadFile(config_file);
        std::string topic_name = config["video_stream_topic"].as<std::string>();

        // 使用读取的主题名称订阅
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name, 10, std::bind(&VideoSubscriber::topic_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        try {
            // 将ROS图像消息转换为OpenCV图像
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // 显示图像
            cv::imshow("Video Stream", frame);
            cv::waitKey(10);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // // 确保提供了配置文件路径
    // if (argc < 2) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run <package_name> video_subscriber <config_file.yaml>");
    //     return 1;
    // }

    std::string config_file = "src/video_streaming_pkg/config/config.yaml";
    auto node = std::make_shared<VideoSubscriber>(config_file);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
