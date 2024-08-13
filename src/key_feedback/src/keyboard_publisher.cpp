#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>  // 假设你要用字符串发布键盘事件
#include <ncurses.h>
#include <string>

class KeyboardPublisher : public rclcpp::Node
{
public:
    KeyboardPublisher()
    : Node("keyboard_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard_event", 10);

        // 初始化 ncurses
        initscr();
        cbreak();
        noecho();
        nodelay(stdscr, TRUE);  // 非阻塞模式
        keypad(stdscr, TRUE);

        RCLCPP_INFO(this->get_logger(), "Keyboard Publisher initialized");
    }

    ~KeyboardPublisher()
    {
        // 结束 ncurses 模式
        endwin();
    }

    void run()
    {
        while (rclcpp::ok()) {
            int ch = getch();
            if (ch != ERR) {  // 检测到按键输入
                std_msgs::msg::String msg;
                msg.data = keyToString(ch);
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Key pressed: %s", msg.data.c_str());
            }

            rclcpp::spin_some(this->shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    std::string keyToString(int ch) const
    {
        switch (ch) {
            case KEY_UP: return "UP";
            case KEY_DOWN: return "DOWN";
            case KEY_LEFT: return "LEFT";
            case KEY_RIGHT: return "RIGHT";
            case 27: return "ESC";
            default: return std::string(1, static_cast<char>(ch));
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisher>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
