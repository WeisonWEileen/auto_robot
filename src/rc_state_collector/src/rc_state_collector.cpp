// Copyright 2024 Weison_Pan

#include <rc_state_collector/rc_state_collector.hpp>

namespace rc_state_collector {

StateCollectorNode::StateCollectorNode(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("state_collector", options), 
    robo_mode_(0), 
    area_mode_(0)
  {

  RCLCPP_INFO(this->get_logger(), "StateCollectorNode has been started.");

  // this->declare_parameter<std::vector<double>>(
  //     "desirce_pose", {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});


  rim_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/rc/vision_rimstate",
      10,
      [this](std_msgs::msg::Int32::SharedPtr msg)
      {
        this->rim_mode_ = msg->data;
      }
    );

  // @TODO 优化成只有欧拉角的计算
  //发布的位置的发布点，先

  pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/rc/desire_pose", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&StateCollectorNode::robo_state_callback, this));
}

//总体机器人控制接口，发布最终的运动信息，以30Hz的频率控制机器人
void StateCollectorNode::robo_state_callback() {

  // 发布目标的运动信息，记得z代表的是yaw轴的角度
  geometry_msgs::msg::Point desire_pose_msg;
  desire_pose_msg.x = 3;
  desire_pose_msg.y = 0;
  desire_pose_msg.z = 0 * PI;

  pose_pub_->publish(desire_pose_msg);
}


void StateCollectorNode::carried_state_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {;

    //
    if (msg->data == true) {
   // 如果携带到球了
    robo_mode_ = 2;
  }
  else {
    // 如果还在 1 区
    if (area_mode_ == 0)
      // 更新机器人状态为0：通往3区
      robo_mode_ = 0;
    // 发送去往3区的运动指令;
    // send_goarea3_goal();
    // 如果在 3 区
    else if (area_mode_ == 1) {
      // 更新机器人状态为1：找球
      robo_mode_ = 1;
      // send_findball_goal();
    }
  }
}
} // namespace rc_state_collector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_state_collector::StateCollectorNode)
