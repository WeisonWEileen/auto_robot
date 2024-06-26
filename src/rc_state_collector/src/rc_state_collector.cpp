// Copyright 2024 Weison_Pan

#include <rc_state_collector/rc_state_collector.hpp>

namespace rc_state_collect{

StateCollectorNode::StateCollectorNode(const rclcpp::NodeOptions &options):Node("state_collector",options){
  RCLINFO(this->get_logger() ,"StateCollectorNode has been started.");
    robo_mode_ = 0;
    area_mode_ = 0;
    odom_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/Odometry", 10, std::bind(&StateCollectorNode::odom_callback, this, std::placeholders::_1));





}
StateCollectorNode::odom_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg){
    // 里程计数据判断机器人状态
    // 
    if(msg->pose->x < 8){
        // 发送到3区的goal
        robo_mode_ = 0;
    }
    else{
        // 机器人状态未知
        robo_mode_ = 1;
    }
}

StateCollectorNode::rim_goal_callback(
    const std::msgs::msg::String::SharedPtr msg){
    // 接收框的目标信息, 生成运动数据
    // 更新机器人状态
    robo_mode_ = 2;

}

StateCollectorNode::carried_state_callback(
    const std::msgs::msg::Bool::SharedPtr msg) {
  // 根据携带状态判断机器人状态
  //
  if (msg->data == true) {
    // 如果携带到球了
    robo_mode_ = 2;
  } else {
    // 如果还在 1 区
    if (area_mode_ == 0)
      // 更新机器人状态为0：通往3区
      robo_mode_ = 0
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
}