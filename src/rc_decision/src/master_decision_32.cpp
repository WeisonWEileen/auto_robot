#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include <cstdio>

class MasterDecision : public rclcpp::Node
{
public:
    MasterDecision()
    : Node("master_decision_32"), robot_mode_(0), last_published_robot_mode_(-1)
    {
        pub_ = this->create_publisher<std_msgs::msg::Int64>("robot_mode", 10);
        move_goal_id_pub_ = this->create_publisher<std_msgs::msg::Int64>("move_goal_id", 10);
        nav_goal_id_pub_ = this->create_publisher<std_msgs::msg::Int64>("nav_goal_id", 10);  
        game_sub_ = this->create_subscription<std_msgs::msg::String>("game", 10, 
            std::bind(&MasterDecision::gameStartCallback, this, std::placeholders::_1));
        move_status_sub_ = this->create_subscription<std_msgs::msg::String>("move_status", 10, 
            std::bind(&MasterDecision::moveStatusCallback, this, std::placeholders::_1));
        ball_status_sub_ = this->create_subscription<std_msgs::msg::String>("ball_status", 10, 
            std::bind(&MasterDecision::ballStatusCallback, this, std::placeholders::_1));
        nav_status_sub_ = this->create_subscription<std_msgs::msg::String>("nav_status", 10, 
            std::bind(&MasterDecision::navStatusCallback, this, std::placeholders::_1));  // 新增的订阅器

        // 发布初始状态
        publishMode();
    }

private:
    void gameStartCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "start") {
            robot_mode_ = 1;
            publishNavGoalId(0);
            // printf("robot_mode = 1\n");
        }
        publishMode();
    }

    void navStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "succeed" && (robot_mode_ == 1)) {
            robot_mode_ = 2;
            // printf("robot_mode = 2\n");
        }
        publishMode();
    }

    void moveStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "OK") {
            if (robot_mode_ == 6) {
                robot_mode_ = 2;
                // printf("robot_mode = 2\n");
            } else if (robot_mode_ == 4) {
                robot_mode_ = 5;
                // printf("robot_mode = 5\n");
            }
        }
        publishMode();
    }

    void ballStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "picked_up" && robot_mode_ == 2) {
            robot_mode_ = 4;
            // printf("robot_mode = 4\n");
        } else if (msg->data == "put_down" && robot_mode_ == 5) {
            robot_mode_ = 6;
            // printf("robot_mode = 6\n");
            publishMoveGoalId(0);
        } else if (msg->data == "reset") {
            robot_mode_ = 0;
            // printf("robot_mode = 0\n");
        }
        publishMode();
    }

    void publishMode() {
        if (robot_mode_ != last_published_robot_mode_) {
            std_msgs::msg::Int64 mode_msg;
            mode_msg.data = robot_mode_;
            pub_->publish(mode_msg);
            RCLCPP_INFO(this->get_logger(), "Publish robot mode: %d", robot_mode_);
            last_published_robot_mode_ = robot_mode_;
        }
    }

    void publishNavGoalId(int64_t id) {
        std_msgs::msg::Int64 msg;
        msg.data = id;
        nav_goal_id_pub_->publish(msg);
    }

    void publishMoveGoalId(int64_t id) {
        std_msgs::msg::Int64 msg;
        msg.data = id;
        move_goal_id_pub_->publish(msg);
    }

    int robot_mode_;
    int last_published_robot_mode_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr move_goal_id_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr nav_goal_id_pub_;  
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr game_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr move_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ball_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;  
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterDecision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}