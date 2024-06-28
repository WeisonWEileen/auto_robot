// pid_control_server.hpp
#ifndef RC_GOAL_SERVER_HPP_
#define RC_GOAL_SERVER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include 
#include "rc_controller/pid_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define RADIUS_TOLERANCE 0.05

namespace rc_state_collector {
class NavToPoseServer : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit NavToPoseServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped target_pose_;
  // rc_controller::PIDController pid_x_, pid_y_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void control_loop();

  double get_current_x_position();

  double get_current_y_position();

  // mid360的里程计输出订阅
  nav_msgs::msg::Odometry::SharedPtr cur_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

  //用于存放当前的目标位置，直接订阅mid360的里程计输出

  // void  init_PID();
  // std::unique_ptr<rc_controller::PIDController> x_controller_;
  // std::unique_ptr<rc_controller::PIDController> y_controller_;
  // std::unique_ptr<rc_controller::PIDController> yaw_controller_;
}; // namespace rclcpp::Node

#endif // PID_CONTROL_SERVER_HPP_
}