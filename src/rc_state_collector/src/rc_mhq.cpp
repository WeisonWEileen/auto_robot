// velocity_publisher.cpp
#include "rc_state_collector/rc_mhq.hpp"

namespace rc_state_collector {
MHQ_R2::MHQ_R2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("MhqR2Node", options) {

  RCLCPP_INFO(this->get_logger(), "MHQR2 has been started.");
  // go3_cmd_vx time(ms) go3_cmd_vy time(ms) go3_cmd_vx time
  this->declare_parameter<std::vector<float>>(
      "go3_params_", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  go3_params_ = this->get_parameter("go3_params_").as_double_array();

  publisher_ =
      this->create_publisher<rc_interface_msgs::msg::Mhq>("/cmd_vel", 10);

  // milliseconds to pubisher
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                   std::bind(&MHQ_R2::timer_callback, this));

  carrayState_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/rc_decision/carry_state", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg)
      {
          carried_state_ = msg->data;
      });

  //   receive_thread_ = std::thread(&MHQ_R2::carried_state_update,this);
}

void MHQ_R2::publish_velocity() { publisher_->publish(velocity_msg_); }



void MHQ_R2::timer_callback() {
  static int count = 0;
  count++;
    // forward
  if (count <= go3_params_[0]) {
    velocity_msg_.cmd_vx = go3_params_[1];
    velocity_msg_.cmd_vy = 0.0f;
    velocity_msg_.desire_yaw = 0.0;
    velocity_msg_.mode = 0;
  }
  // translation
  else if (count > (go3_params_[0]) && count <= (go3_params_[0] + go3_params_[2])) {
    velocity_msg_.cmd_vx = 0.0f;
    velocity_msg_.cmd_vy = go3_params_[3];
    velocity_msg_.desire_yaw = 0.0;
    velocity_msg_.mode = 0;
  } else if (count > (go3_params_[0] + go3_params_[2]) &&
             count <= (go3_params_[2] + go3_params_[4])) {
    velocity_msg_.cmd_vx = go3_params_[5];
    velocity_msg_.cmd_vy = 0;
    velocity_msg_.desire_yaw = 0.0;
    velocity_msg_.mode = 0;
  }

  else {
    velocity_msg_.cmd_vx = 0;
    velocity_msg_.cmd_vy = 0;
    velocity_msg_.desire_yaw = 0.0;
    velocity_msg_.mode = 0;
    timer_.reset();
    // After going to Area 1, and reset this chasis controller
    // count = 0;
  }
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "go 3 velocity: " << velocity_msg_.cmd_vx << " "
                                       << velocity_msg_.cmd_vy << " "
                                       << velocity_msg_.desire_yaw);
  publish_velocity();
}


} // namespace rc_state_collector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_state_collector::MHQ_R2)
