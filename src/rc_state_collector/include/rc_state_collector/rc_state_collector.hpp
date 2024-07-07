// Copyright 2024 Weison_Pan

#ifndef RC_STATE_COLLECTOR_HPP_
#define RC_STATE_COLLECTOR_HPP_
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rc_controller/pid_controller.hpp"
#include "rc_interface_msgs/msg/motion.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
// #include "yolov8_msgs/msg/key_point3_d.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"

#define PI 3.1415926

// 用于接受所有抽象层的数据，生成最终的机器人的控制策略

#define ROBO_MODE_RUNNING 0
#define ROBO_MODE_FINDING 1
#define ROBO_MODE_PLACING 2

#define AREA_MODE_OUTSIDE3 0
#define AREA_MODE_INSIDE3 1

#define CARRY_BALL_MODE_NO 0
#define CARRY_BALL_MODE_YES 1

#define O_0_E_1 1
#define O_0_E_0 3
#define O_1_E_1 5
#define O_0_E_2 6

namespace rc_state_collector {
class StateCollectorNode : public rclcpp::Node {
public:
  StateCollectorNode(const rclcpp::NodeOptions &options);

  // 整个机器人的状态
  // ROBO_MODE_GO3 通往3区状态： 0到1
  // ROBO_MODE_GO3 通往3区状态： 1到2
  // ROBO_MODE_GO3 通往3区状态： 2到3

  // ROBO_MODE_FINDBALL        4
  
  // ROBO_MODE_PUTBALL         5

  // @TODO 这里还要写一个重试的接口
  // ROBO_MODE_FINDING 找球状态：1
  // ROBO_MODE_PLACING 放球装填：2
  int robo_mode_;

  // 机器人所处的位置的装填
  // AREA_MODE_0 1区
  // AREA_MODE_1 2区
  // AREA_MODE_3 3区
  // robo_mode_ 应该优先考虑area_mode_而不用考虑其它地方
  int area_mode_;

  // 球框的状态
  int rim_mode_;

  // 机器人是否携带球
  // CARRY_BALL_MODE_NO 未携带球：0
  // CARRY_BALL_MODE_YES 携带球：1
  int carry_ball_mode_;
  

private:
  // 调参接口，获取所有的3个点的位置参数
  void getParam();

  //直接订阅rim的state
  //    --------------  //
  // 更新目标框的发布生成运动信息
  //   ---------------  //
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rim_state_sub_;

  // 订阅球的携带状态
  // 两种状态，对应有或者无，对应气压泵里面的气球的状态更新
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr carried_state_sub_;
  void carried_state_callback(const std_msgs::msg::Bool::SharedPtr msg);

  geometry_msgs::msg::Point desire_pose_msg1_; //上三区的第一个目标点
  geometry_msgs::msg::Point desire_pose_msg2_; //上三区的第二个目标点
  geometry_msgs::msg::Point desire_pose_msg3_; //上三区的第三个目标点
  geometry_msgs::msg::Point desire_pose_msg4_; //找球的目标点，这里要求正对球框
  geometry_msgs::msg::Point desire_pose_msg5_;
  geometry_msgs::msg::Point desire_pose_msg6_;

  // 订阅目标球的 realsense 三维坐标，但是只取第二个维度的
  rclcpp::Subscription<yolov8_msgs::msg::KeyPoint3DArray>::SharedPtr
  realsense_ball_sub_;
  std::array<float, 2> realsense_ball_;

  // 订阅v4l2的 鱼眼相机的三维坐标，这是用来找球的
  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr
  v4l2_ball_sub_;
  std::array<float, 2> v4l2_ball_;

  // 订阅气压泵串口来的是否订阅到了球
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attach_state_sub_;
  uint8_t attach_state_;

  // 订阅位置模式
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr position_mode_sub;

  // 定时发布机器人的运动信息
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rc_interface_msgs::msg::Motion>::SharedPtr motion_pub_;



  void robo_state_callback();

  // // 用于订阅一开始的位置状态
  // rclcpp::Subscription<const sensor_msgs::msg::Image::SharedPtr>
  //     ares_detector_sub_;
};
} // namespace rc_state_collector

#endif