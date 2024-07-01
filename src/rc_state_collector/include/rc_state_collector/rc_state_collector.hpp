// Copyright 2024 Weison_Pan

#ifndef RC_STATE_COLLECTOR_HPP_
#define RC_STATE_COLLECTOR_HPP_
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>

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
    class StateCollectorNode : public rclcpp::Node
    {
        public:
            StateCollectorNode(const rclcpp::NodeOptions &options);

            // 整个机器人的装填
            // ROBO_MODE_RUNNING 通往3区状态： 0
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
            //直接订阅rim的state
            //    --------------  //
            // 更新目标框的发布生成运动信息
            //   ---------------  //
          rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rim_state_sub_;

          // 订阅球的携带状态
          // 两种状态，对应有或者无
          rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
              carried_state_sub_;
          void carried_state_callback(const std_msgs::msg::Bool::SharedPtr msg);

          rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
              pose_pub_;

          // 定时更新机器人装填
          rclcpp::TimerBase::SharedPtr timer_;
          void robo_state_callback();


    };
}

#endif