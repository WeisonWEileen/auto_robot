#include "rc_controller/pid_controller.hpp"

namespace rc_controller {

PoseControllerNode::PoseControllerNode(const rclcpp::NodeOptions &options)
    : Node("pose_controller", options) {
  RCLCPP_INFO(this->get_logger(), "Starting PoseController node!");

  init_PID();

  // Fastlio
  //实时订阅Mid360发出来的当前位姿信息
  poseUpdate_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::SensorDataQoS(),
      std::bind(&PoseControllerNode::poseUpdate_callback, this,
                std::placeholders::_1));
  // State
  // 订阅目标位姿信息
  poseCommand_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/rc/desire_pose", rclcpp::SensorDataQoS(),
      std::bind(&PoseControllerNode::poseCommand_callback, this,
                std::placeholders::_1));

  // 发布电机控制，和下位机对接点
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void PoseControllerNode::init_PID() {

  this->declare_parameter<std::vector<double>>("x_controller_",
                                               {0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("y_controller_",
                                               {0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("yaw_controller_",
                                               {0.0, 0.0, 0.0, 0.0, 0.0});

  this->declare_parameter<int>("scale_factor", 1);
  scale_factor_ = this->get_parameter("scale_factor").as_double();

  std::vector<double> x_pid_param =
      this->get_parameter("x_controller_").as_double_array();
  std::vector<double> y_pid_param =
      this->get_parameter("y_controller_").as_double_array();
  std::vector<double> yaw_pid_param =
      this->get_parameter("z_controller_").as_double_array();

  // @TODO yaml添加pid初始化结构体的struct定义
  x_controller_ = std::make_unique<PIDController>(x_pid_param);
  y_controller_ = std::make_unique<PIDController>(y_pid_param);
  yaw_controller_ = std::make_unique<PIDController>(yaw_pid_param);
}

// 订阅mid360的驱动接口
void PoseControllerNode::poseUpdate_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  // 先直接获取x和y
  auto current_x = msg->pose.pose.position.x;
  auto current_y = msg->pose.pose.position.y;

  //订阅四元数的角度，转换为yaw轴的角度
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f", current_x, current_y,
              yaw);

  x_controller_->setMeasure(current_x);
  y_controller_->setMeasure(current_y);
  yaw_controller_->setMeasure(yaw);
}

void PoseControllerNode::poseCommand_callback(
    const geometry_msgs::msg::Point::ConstSharedPtr msg) {
  // do something with the pose
  // call the PID controllers
  // publish the control commands

  // @TODO 后续可以取消
  RCLCPP_INFO_STREAM(this->get_logger(), "Desired x: " << msg->x<< ", y: " << msg->y << ", yaw: " << msg->z);

  geometry_msgs::msg::Twist twist_msg;

  

  // 注意这里z是直接对应的yaw轴的转定角度
  twist_msg.linear.x = x_controller_->pidCalculate(msg->x);
  twist_msg.linear.y = y_controller_->pidCalculate(msg->y);
  twist_msg.angular.z = yaw_controller_->pidCalculate(msg->z);

  // @TODO 后续可以取消
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Output_of_chasis x: " << twist_msg.linear.x
                                            << ", y: " << twist_msg.linear.y
                                            << ", yaw: " << twist_msg.linear.z);

  cmd_pub_->publish(twist_msg);
}

PIDController::PIDController(std::vector<double> pid_param) {
  kp_ = pid_param[0];
  ki_ = pid_param[1];
  kd_ = pid_param[2];
  maxOut_ = pid_param[3];
  integral_lim_ = pid_param[4];
}

double PIDController::pidCalculate(double target) {
  //注意measure_在另外一个线程中得到更新

  err_ = target - measure_;
  Pout_ = kp_ * err_;
  ITerm_ = ki_ * err_;
  Iout_ += ITerm_;

  // 积分限幅
  if (Iout_ > integral_lim_) {
    Iout_ = integral_lim_;
  } else if (Iout_ < -integral_lim_) {
    Iout_ = integral_lim_;
  }

  Dout_ = kd_ * (err_ - last_err_);
  output_ = Pout_ + Iout_ + Dout_;

  // 输出限幅
  if (output_ > maxOut_) {
    output_ = output_;
  } else if (Iout_ < -output_) {
    output_ = output_;
  }

  last_err_ = err_;
  // last_measure_ = measure_;
  last_dout_ = Dout_;

  return output_;
}
void PIDController::setMeasure(double measure) { measure_ = measure; }
} // namespace rc_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_controller::PoseControllerNode)
