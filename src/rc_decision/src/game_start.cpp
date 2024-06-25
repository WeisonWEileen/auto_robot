#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

class GameStartPublisher : public rclcpp::Node
{
public:
    GameStartPublisher()
    : Node("game_start"), should_publish_(true)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("game", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Int64>(
            "robot_mode",
            10,
            [this](std_msgs::msg::Int64::SharedPtr msg) {
                if (msg->data != 0) {
                    should_publish_ = false;
                }
            }
        );
        publishGameStart();
    }

private:
    void publishGameStart()
    {
        std_msgs::msg::String msg;
        msg.data = "start";
        while (rclcpp::ok() && should_publish_) {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
    bool should_publish_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GameStartPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}