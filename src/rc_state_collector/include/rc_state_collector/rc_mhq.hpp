#ifndef RC_MHQ_HPP_
#define RC_MHQ_HPP_

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rc_interface_msgs/msg/mhq.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>

namespace rc_state_collector {

class MHQ_R2 : public rclcpp::Node {
public:
  MHQ_R2(const rclcpp::NodeOptions &options);

private:
    // to store tuning parameters for go3
    std::vector<double> go3_params_;

  void publish_velocity();
  void timer_callback();

  rclcpp::Publisher<rc_interface_msgs::msg::Mhq>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
    //
  rc_interface_msgs::msg::Mhq velocity_msg_;

  //   to detect whether carried ball

  uint8_t carried_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr carrayState_sub_;
  //   std::thread carried_state_thread_;
  //   void carried_state_update();
};
} // namespace rc_state_collector
#endif