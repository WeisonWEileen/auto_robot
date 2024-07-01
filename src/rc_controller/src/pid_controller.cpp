#include "rc_controller/pid_controller.hpp"

#define Area_12_XThres 6.0
#define Area_22_YThres -3.5
#define Area_23_XThres 9.0

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

  //发布位置的信息
  position_mode_.data = 0;
  position_mode_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("/rc/position_mode", 10);

  // 订阅rc_state_collector目标位姿信息
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
  this->declare_parameter<double>("scale_factor_", 1.0);


  std::vector<double> x_pid_param =
      this->get_parameter("x_controller_").as_double_array();
  std::vector<double> y_pid_param =
      this->get_parameter("y_controller_").as_double_array();
  std::vector<double> yaw_pid_param =
      this->get_parameter("yaw_controller_").as_double_array();
  scale_factor_ = this->get_parameter("scale_factor_").as_double();

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

  // 根据现在的位置直接发布位置模式

  // 如果机器人在1区
  if ( position_mode_.data == 0 ) {
    if (current_x < Area_12_XThres && current_y < Area_22_YThres) {
    // 还在跑第一段线段,继续跑
      position_mode_.data = 0;
      position_mode_pub_->publish(position_mode_);
    } else if (current_x > Area_12_XThres && current_y < Area_22_YThres) {
      // 已经跑完
      position_mode_.data = 1;
      position_mode_pub_->publish(position_mode_);
    }
  } else if (current_x > 8 && current_y > 4) {
    position_mode_.data = 1;
    position_mode_pub_->publish(position_mode_);
  }

  //订阅四元数的角度，转换为yaw轴的角度
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  RCLCPP_WARN(this->get_logger(), "x: %f, y: %f, yaw: %f", current_x, current_y,
              yaw);

  current_pose_.x = current_x;
  current_pose_.y = current_y;
  current_pose_.yaw = yaw;

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
  RCLCPP_INFO_STREAM(this->get_logger(), "Input_Desired x: " << msg->x<< ", y: " << msg->y << ", yaw: " << msg->z);

  geometry_msgs::msg::Twist twist_msg;


  //给定当前的yaw，转换为世界坐标系下的desire_x，desire_y的平移速度，desire_yaw的不需要改
  // 这里面是已经算法差值的，所以下面输出控制器的时候不需要再给measure，直接给measure 0.0
  Pose world_target_pose = target_xy_transform(msg->x, msg->y, msg->z);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "After cordinate transform x: " << world_target_pose.x
                                         << ", y: " << world_target_pose.y
                                         << ", yaw: " << world_target_pose.yaw);

  // 注意这里z是直接对应的yaw轴的转定角度
  twist_msg.linear.x = x_controller_->pidCalculate(0.0, world_target_pose.x);
  twist_msg.linear.y = y_controller_->pidCalculate(0.0, world_target_pose.y);
  twist_msg.angular.z = yaw_controller_->pidCalculate(0.0, world_target_pose.yaw);

  // @TODO 后续可以取消
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Output_of_chasis x: " << twist_msg.linear.x
                                            << ", y: " << twist_msg.linear.y
                                            << ", yaw: " << twist_msg.angular.z);

  cmd_pub_->publish(twist_msg);
}

PIDController::PIDController(std::vector<double> pid_param) {
  kp_ = pid_param[0];
  ki_ = pid_param[1];
  kd_ = pid_param[2];
  maxOut_ = pid_param[3];
  integral_lim_ = pid_param[4];
}

double PIDController::pidCalculate(double current, double desire_value) {
  //注意measure_在另外一个线程中得到更新

  err_ = desire_value - current;
  Pout_ = kp_ * err_;
  ITerm_ = ki_ * err_;
  Iout_ += ITerm_;

  // 积分限幅
  if (Iout_ > integral_lim_) {
    Iout_ = integral_lim_;
  } else if (Iout_ < -integral_lim_) {
    Iout_ = -integral_lim_;
  }

  Dout_ = kd_ * (err_ - last_err_);
  output_ = Pout_ + Iout_ + Dout_;

  // 输出限幅
  if (output_ > maxOut_) {
    output_ = maxOut_;
  } else if (output_ < -maxOut_) {
    output_ = -maxOut_;
  }

  last_err_ = err_;
  // last_measure_ = measure_;
  last_dout_ = Dout_;

  return output_;
}
void PIDController::setMeasure(double measure) { measure_ = measure; }

Pose PoseControllerNode::target_xy_transform(double desire_world_x,
                                             double desire_world_y,
                                             double desire_yaw) {

  Pose world_target_pose;

  world_target_pose.x =
      cos(current_pose_.yaw) * (desire_world_x - current_pose_.x) -
      sin(current_pose_.yaw) * (desire_world_y - current_pose_.y);
  world_target_pose.y = sin(current_pose_.yaw) * (desire_world_x-current_pose_.x) + cos(current_pose_.yaw) * (desire_world_y - current_pose_.y);

  world_target_pose.yaw = desire_yaw - current_pose_.yaw;
  if (world_target_pose.yaw > Pai) {
    world_target_pose.yaw -= Pai;
  } else if (world_target_pose.yaw < -Pai) {
    world_target_pose.yaw += Pai;
  }

    return world_target_pose;
}

} // namespace rc_controller
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_controller::PoseControllerNode)
