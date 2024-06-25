#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>  
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double min_speed, double max_speed)
        : kp_(kp), ki_(ki), kd_(kd), min_speed_(min_speed), max_speed_(max_speed), integral_(0.0), prev_error_(0.0) {}

    double control(double setpoint, double pv)
    {
        double error = setpoint - pv;
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;
        return limit(kp_ * error + ki_ * integral_ + kd_ * derivative);
    }

private:
    double limit(double value)
    {
        if (value > max_speed_) {
            return max_speed_;
        } else if (value < -max_speed_) {
            return -max_speed_;
        } else if (value > 0 && value < min_speed_) {
            return min_speed_;
        } else if (value < 0 && value > -min_speed_) {
            return -min_speed_;
        } else {
            return value;
        }
    }

    double kp_, ki_, kd_, min_speed_, max_speed_, integral_, prev_error_;
};

struct TargetPose {
    double x, y, yaw;
    bool operator==(const TargetPose& other) const {
        return x == other.x && y == other.y && yaw == other.yaw;
    }
};

class PoseController : public rclcpp::Node
{
public:
    PoseController()
        : Node("rc_move"),
            x_controller_(0.12, 0.0, 0.12, 0.01, 0.15),
            y_controller_(0.12, 0.0, 0.12, 0.01, 0.15),
            yaw_controller_(0.15, 0.0, 0.0, 0.0, 0.15)
    { 
        target_poses_ = {
            {1.5, -0.5, -M_PI/2},
            {-0.020, 1.55, -M_PI/2},
            {0.735, 1.55, -M_PI/2},
            {1.482, 1.55, -M_PI/2},
            {2.234, 1.55, -M_PI/2},
            {2.981, 1.55, -M_PI/2},
        };
        target_ = target_poses_[0];

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "filtered_pose", 10, std::bind(&PoseController::pose_callback, this, std::placeholders::_1));
        cmd_pid_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_move", 10);
        move_goal_id_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "move_goal_id", 10, std::bind(&PoseController::move_goal_id_callback, this, std::placeholders::_1));
        move_status_publisher_ = this->create_publisher<std_msgs::msg::String>("move_status", 10); 
    }

    void move_goal_id_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        size_t id = static_cast<size_t>(msg->data);
        if (id < target_poses_.size()) {
            target_ = target_poses_[id];
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid position id: %ld", id);
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.orientation, quat);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f", x, y, yaw);
        TargetPose target_local = transformTargetPose(x, y, yaw);
        calculateControl(target_local);
    }

private:
    TargetPose transformTargetPose(double x, double y, double yaw)
    {
        TargetPose target_local;
        target_local.x = cos(yaw) * (target_.x - x) + sin(yaw) * (target_.y - y);
        target_local.y = -sin(yaw) * (target_.x - x) + cos(yaw) * (target_.y - y);
        target_local.yaw = target_.yaw - yaw;
        if (target_local.yaw > M_PI)
            target_local.yaw -= 2 * M_PI;
        else if (target_local.yaw < -M_PI)
            target_local.yaw += 2 * M_PI;
        return target_local;
    }

    void calculateControl(const TargetPose &target_local)
    {
        double error_threshold = 0.03;
        double control_x = 0.0;
        double control_y = 0.0;
        double control_yaw = 0.0;
        if (std::abs(target_local.x) > error_threshold || std::abs(target_local.y) > error_threshold || std::abs(target_local.yaw) > error_threshold) {
            control_x = x_controller_.control(0.0, target_local.x);
            control_y = y_controller_.control(0.0, target_local.y);
            control_yaw = yaw_controller_.control(0.0, target_local.yaw);
        } else {
            // Publish "OK" when the robot is near the target
            std_msgs::msg::String msg;
            msg.data = "OK";
            move_status_publisher_->publish(msg);
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = control_x;
        cmd.linear.y = -control_y;
        cmd.angular.z = control_yaw;
        cmd_pid_publisher_->publish(cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pid_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_status_publisher_; 
    std::vector<TargetPose> target_poses_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr move_goal_id_subscriber_;
    PIDController x_controller_, y_controller_, yaw_controller_;
    TargetPose target_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}