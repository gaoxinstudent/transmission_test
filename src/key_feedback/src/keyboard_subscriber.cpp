#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class KeyboardSubscriber : public rclcpp::Node
{
public:
    KeyboardSubscriber()
    : Node("keyboard_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "keyboard_event", 10, std::bind(&KeyboardSubscriber::topic_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Keyboard Subscriber initialized");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received key event: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
