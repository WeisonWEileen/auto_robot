// Copyright 2024 Weison_Pan

#include <rc_state_collector/rc_state_collector.hpp>

namespace rc_state_collector {

StateCollectorNode::StateCollectorNode(const rclcpp::NodeOptions &options):Node("state_collector",options){
  RCLCPP_INFO(this->get_logger() ,"StateCollectorNode has been started.");

  this->declare_parameter<std::vector<double>>("desirce_pose",
                                               {1.0, 0.0, 0.0, 0.0, 0.0,0.0,1.0});


  robo_mode_ = 0;
  area_mode_ = 0;
  // odom_sub_ =
  // this->create_subscription<geometry_msgs::msg::TransformStamped>(
  //     "/Odometry", 10, std::bind(&StateCollectorNode::odom_callback, this,
  //     std::placeholders::_1));

  pose_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/rc/decision", 10);

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&StateCollectorNode::robo_state_callback, this));
}

void StateCollectorNode::robo_state_callback()
 {
  std::vector<double> desire_pose_param =
      this->get_parameter("x_controller_").as_double_array();

  nav_msgs::msg::Odometry msg;
  msg.pose.pose.position.x =   desire_pose_param[0];
  msg.pose.pose.position.y =    desire_pose_param[1];
  msg.pose.pose.orientation.x = desire_pose_param[2];
  msg.pose.pose.orientation.y = desire_pose_param[3];
  msg.pose.pose.orientation.z = desire_pose_param[4];
  msg.pose.pose.orientation.w = desire_pose_param[5];

  pose_pub_->publish(msg);
 }

void StateCollectorNode::odom_callback(
    const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
  // 里程计数据判断机器人状态
  //
  // if(msg->pose->x < 8){
  //     // 发送到3区的goal
  //     robo_mode_ = 0;
  // }
  // else{
  //     // 机器人状态未知
  //     robo_mode_ = 1;
  // }
}

void StateCollectorNode::rim_goal_callback(
    const std_msgs::msg::Int32::SharedPtr msg) {
  // 接收框的目标信息, 生成运动数据
  // 更新机器人状态
  robo_mode_ = 2;
}

void StateCollectorNode::carried_state_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  // 根据携带状态判断机器人状态
  //
  if (msg->data == true) {
    // 如果携带到球了
    robo_mode_ = 2;
  } else {
    // 如果还在 1 区
    if (area_mode_ == 0)
      // 更新机器人状态为0：通往3区
      robo_mode_ = 0;
    // 发送去往3区的运动指令;
    // send_goarea3_goal();
    // 如果在 3 区
    else if (area_mode_ == 1)
    {
      // 更新机器人状态为1：找球
      robo_mode_ = 1;
      // send_findball_goal();
      }
    }
  }
  } // namespace rc_state_collector


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_state_collector::StateCollectorNode)
