#ifndef RC_CONTROLLER_PID_CONTROLLER_HPP_
#define RC_CONTROLLER_PID_CONTROLLER_HPP_

// 这是控制层的PID控制器，用于控制机器人的运动

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace rc_controller {

class PIDController {
public:
  PIDController(std::vector<double>);

  // double kp, double ki, double kd,double max_out, double integral_lim

  double pidCalculate(double desire_value);
  void setMeasure(double measure);

  double measure_ = 0.0;
private:
  double last_measure_ = 0.0;
  double err_ = 0.0;
  double last_err_ = 0.0;
  double last_dout_ = 0.0;

  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;

  double ITerm_ = 0.0;

  double Pout_;
  double Iout_;
  double Dout_;

  double maxOut_;
  double integral_lim_;

  double output_ = 0.0;
};

//@TODO 目前的计算是desire value 和 current value
//分开两部分传输，不知道会不会有问题，后续可以考虑使用 message filter
//进行软同步

class PoseControllerNode : public rclcpp::Node {
public:
  PoseControllerNode(const rclcpp::NodeOptions &options);

private:
  std::unique_ptr<PIDController> x_controller_;
  std::unique_ptr<PIDController> y_controller_;
  std::unique_ptr<PIDController> yaw_controller_;
  int robot_mode_;

  //初始化PID嵌套类
  void init_PID();

  // 发布运动控制指令
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // 订阅现在位置
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      poseUpdate_sub_;
  void poseUpdate_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // 订阅目标位置,z直接对应yaw轴
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
      poseCommand_sub_;
  void
  poseCommand_callback(const geometry_msgs::msg::Point::ConstSharedPtr msg);

  float scale_factor_;
};
//         rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
//         cmd_pid_publisher_;

// }

} // namespace rc_controller



#endif