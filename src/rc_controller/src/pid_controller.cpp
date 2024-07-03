#include "rc_controller/pid_controller.hpp"

#define Area_12_XThres 6.0
#define Area_22_YThres -3.5
#define Area_23_XThres 9.0

// 终端日志
// 红色：对应的是雷达位置更新信息
// 黄色：对应的是PID控制器的输出
// 灰色：对应的是rc_state_collector给的命令

namespace rc_controller {

PoseControllerNode::PoseControllerNode(const rclcpp::NodeOptions &options)
    : Node("pose_controller", options) {
  RCLCPP_INFO(this->get_logger(), "Starting PoseController node!");

  get_desireLoc();
  init_PID();

  // 初始化位置
  current_pose_.x = 0;
  current_pose_.y = 0;
  current_pose_.yaw = 0;



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

  std::vector<double> x_pid_param =
      this->get_parameter("x_controller_").as_double_array();
  std::vector<double> y_pid_param =
      this->get_parameter("y_controller_").as_double_array();
  std::vector<double> yaw_pid_param =
      this->get_parameter("yaw_controller_").as_double_array();

  // @TODO yaml添加pid初始化结构体的struct定义
  x_controller_ = std::make_unique<PIDController>(x_pid_param);
  y_controller_ = std::make_unique<PIDController>(y_pid_param);
  yaw_controller_ = std::make_unique<PIDController>(yaw_pid_param);
}

// 
void PoseControllerNode::get_desireLoc(){

  // 距离到达的阈值

  // desire Pose 1
  this->declare_parameter<std::vector<double>>("desire_pose1", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose1 =
      this->get_parameter("desire_pose1").as_double_array();
  desire_pose1_.x =   desire_pose1[0];
  desire_pose1_.y =   desire_pose1[1];
  desire_pose1_.yaw = desire_pose1[2];
  
  
  // desire Pose 2
  this->declare_parameter<std::vector<double>>("desire_pose2", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose2 =
      this->get_parameter("desire_pose2").as_double_array();
  desire_pose2_.x = desire_pose2[0];
  desire_pose2_.y = desire_pose2[1];
  desire_pose2_.yaw = desire_pose2[2];
 
  // desire Pose 3 
  this->declare_parameter<std::vector<double>>("desire_pose3", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose3 =
      this->get_parameter("desire_pose3").as_double_array();
  desire_pose3_.x = desire_pose3[0];
  desire_pose3_.y = desire_pose3[1];
  desire_pose3_.yaw = desire_pose3[2];
}

// 订阅mid360的驱动接口
void PoseControllerNode::poseUpdate_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  // 先直接获取x和y
  auto current_x = msg->pose.pose.position.x;
  auto current_y = msg->pose.pose.position.y;

  // 根据现在的位置直接发布位置模式

  

  //订阅四元数的角度，转换为yaw轴的角度
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = yaw*180.0f/3.1415926f;

  // 只是想要红色的输出，并不是ERROR
  RCLCPP_ERROR(this->get_logger(), "lidar x: %f, y: %f, yaw: %f", current_x,
               current_y, yaw);

  // 机器人区域状态切换
  // 如果机器人在1区
  if (position_mode_.data == 0) {
    if (euclidis(current_x,current_y,yaw,desire_pose1_.x,desire_pose1_.y,desire_pose1_.yaw) < euclidisThres_ ) {
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
  

  current_pose_.x = current_x;
  current_pose_.y = current_y;
  current_pose_.yaw = yaw;

  }
}

inline double PoseControllerNode::euclidis(double x1, double x2, double x3, double y1, double y2, double y3) {
  return std::sqrt(std::pow(x1 - y1, 2) + std::pow(x2 - y2, 2) + std::pow(x3 - y3, 2));
}

void PoseControllerNode::poseCommand_callback(
    const geometry_msgs::msg::Point::ConstSharedPtr msg) {
  // do something with the pose
  // call the PID controllers
  // publish the control commands

  // @TODO 后续可以取消
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Input_Desired x: " << msg->x << ", y: " << msg->y
                                         << ", yaw: " << msg->z);

  geometry_msgs::msg::Twist twist_msg;

  // 注意这里z是直接对应的yaw轴的转定角度
  // twist_msg.linear.x = x_controller_->pidCalculate(0.0, world_target_pose.x);
  twist_msg.linear.x = x_controller_->pidCalculate(current_pose_.x, msg->x);
  // twist_msg.linear.y = y_controller_->pidCalculate(0.0, world_target_pose.y);
  twist_msg.linear.y = y_controller_->pidCalculate(current_pose_.y, msg->y);
  // measure_yaw,测量到的角度
  twist_msg.linear.z = current_pose_.yaw;
  // desire_yaw，目标值
  twist_msg.angular.z = msg->z;

  // @TODO 后续可以取消
  RCLCPP_WARN_STREAM(this->get_logger(),
                     "Output_of_v x: "
                         << twist_msg.linear.x << ", y: " << twist_msg.linear.y
                         << ", desire_yaw: " << twist_msg.angular.z
                         << ", measure_yaw: " << twist_msg.linear.z);

  cmd_pub_->publish(twist_msg);
}

PIDController::PIDController(std::vector<double> pid_param) {
  kp_ = pid_param[0];
  ki_ = pid_param[1];
  kd_ = pid_param[2];
  integral_lim_ = pid_param[3];
  maxOut_ = pid_param[4];
}

double PIDController::pidCalculate(double current, double desire_value) {

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
  last_dout_ = Dout_;

  return output_;
}

Pose PoseControllerNode::target_xy_transform(double desire_world_x,
                                             double desire_world_y,
                                             double desire_yaw) {

  Pose world_target_pose;

  world_target_pose.x =
      cos(current_pose_.yaw) * (desire_world_x - current_pose_.x) -
      sin(current_pose_.yaw) * (desire_world_y - current_pose_.y);
  world_target_pose.y =
      sin(current_pose_.yaw) * (desire_world_x - current_pose_.x) +
      cos(current_pose_.yaw) * (desire_world_y - current_pose_.y);

  world_target_pose.yaw = desire_yaw - current_pose_.yaw;
  if (world_target_pose.yaw > Pai) {
    world_target_pose.yaw -= 2 * Pai;
  } else if (world_target_pose.yaw < -Pai) {
    world_target_pose.yaw += 2 * Pai;
  }

  return world_target_pose;
}

} // namespace rc_controller
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_controller::PoseControllerNode)
