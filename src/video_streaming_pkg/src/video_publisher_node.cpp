#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher(const std::string &config_file)
    : Node("video_publisher")
    {
        // 读取 YAML 配置文件
        YAML::Node config = YAML::LoadFile(config_file);
        std::string topic_name = config["video_stream_topic"].as<std::string>();
        int delay_ms = config["publish_delay_ms"].as<int>();

        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&VideoPublisher::timer_callback, this));

        // 打开默认摄像头
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        }

        RCLCPP_INFO(this->get_logger(), "Publishing on topic: %s with delay: %d ms", topic_name.c_str(), delay_ms);
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Publishing video frame");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string config_file = "src/video_streaming_pkg/config/config.yaml";
    auto node = std::make_shared<VideoPublisher>(config_file);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
