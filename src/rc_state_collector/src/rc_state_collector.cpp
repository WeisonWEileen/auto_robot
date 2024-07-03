// Copyright 2024 Weison_Pan

#include <rc_state_collector/rc_state_collector.hpp>

namespace rc_state_collector {

StateCollectorNode::StateCollectorNode(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("state_collector", options), robo_mode_(0),
      area_mode_(0), target_ball_({0.0f,0.0f}) {

  RCLCPP_INFO(this->get_logger(), "StateCollectorNode has been started.");

  getParam();


  // 
  rim_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/rc/vision_rimstate", 10, [this](std_msgs::msg::Int32::SharedPtr msg) {
        this->rim_mode_ = msg->data;
      });

  
  // 订阅决策出来的目标球的三维坐标信息,由于球的高度是一定的，所以只用了二维信息
  ball_target_sub_ = this->create_subscription<yolov8_msgs::msg::KeyPoint3D>(
      "/rc_detector/keypoint3d", 10,
      [this](yolov8_msgs::msg::KeyPoint3D::SharedPtr msg)
      {
        target_ball_[0] = msg->point.x;
        target_ball_[1] = msg->point.y;
      });

  // 定时回调发布机器人的运动命令
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(90),
      std::bind(&StateCollectorNode::robo_state_callback, this));

  motion_pub_ = this->create_publisher<rc_interface_msgs::msg::Motion>(
      "/rc/desire_pose", 10);
}

//总体机器人控制接口，发布最终的运动信息，以30Hz的频率控制机器人
void StateCollectorNode::robo_state_callback() {

  // 注意这里的x,y并没有进行解算，在rc_controller里面进行解算
  // 然后结构体里面的 measure_yaw 由controller中得到更新，这里面并没有进行赋值
  rc_interface_msgs::msg::Motion msg;
  msg.cmd_vx = desire_pose_msg1_.x;
  msg.cmd_vy = desire_pose_msg1_.y;
  msg.desire_yaw = desire_pose_msg1_.y;
  msg.ball_x = target_ball_[0];
  msg.ball_y = target_ball_[1];

  motion_pub_->publish(msg);

  // 发布目标的运动信息，记得z代表的是yaw轴的角度
  // 启动的时候如果在1区，那么就是上2区的红色的点的地方

  // if (robo_mode_ == 0) {
  //   geometry_msgs::msg::Point desire_pose_msg;
  //   desire_pose_msg.x = Area_12_XThres;
  //   desire_pose_msg.y = 0;
  //   desire_pose_msg.z = 0 * PI;

  //   pose_pub_->publish(desire_pose_msg_);

  // } else if (robo_mode_ == 1) {
  //   geometry_msgs::msg::Point desire_pose_msg;

  //   desire_pose_msg.x = Area_12_XThres;
  //   desire_pose_msg.y = Area_22_YThres;
  //   desire_pose_msg.z = 0 * PI;

  //   pose_pub_->publish(desire_pose_msg_);
  // }
  // } else if (robo_mode_ == 2) {
  //   geometry_msgs::msg::Point desire_pose_msg;

  //   desire_pose_msg.x = Area_23_XThres;
  //   desire_pose_msg.y = Area_22_YThres;
  //   desire_pose_msg.z = 0 * PI;

  //   pose_pub_->publish(desire_pose_msg_);
  // }

  // } else if (robo_mode_ == 2) {
  //   pose_pub_->publish(desire_pose_msg);
}

void StateCollectorNode::getParam() {
  // 用于测试
  this->declare_parameter<std::vector<double>>("desire_pose", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose =
      this->get_parameter("desire_pose").as_double_array();

  // desire的角度
  desire_pose_msg1_.x = desire_pose[0];
  desire_pose_msg1_.y = desire_pose[1];
  desire_pose_msg1_.z = desire_pose[2];

  this->declare_parameter<std::vector<double>>("desire_pose2", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose2 =
      this->get_parameter("desire_pose2").as_double_array();

  desire_pose_msg2_.x = desire_pose2[0];
  desire_pose_msg2_.y = desire_pose2[1];
  // desire的角度
  desire_pose_msg2_.z = desire_pose2[2];

  this->declare_parameter<std::vector<double>>("desire_pose3", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose3 =
      this->get_parameter("desire_pose3").as_double_array();

  desire_pose_msg3_.x = desire_pose3[0];
  desire_pose_msg3_.y = desire_pose3[1];
  // desire的角度
  desire_pose_msg3_.z = desire_pose3[2];
}

void StateCollectorNode::carried_state_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {

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
