#include "rc_controller/pid_controller.hpp"
#include <chrono>
// #include

#define Area_12_XThres 6.0
#define Area_22_YThres -3.5
#define Area_23_XThres 9.0

// 终端日志
// 红色：对应的是雷达位置更新信息
// 黄色：对应的是PID控制器的输出
// 灰色：对应的是rc_state_collector给的命令

namespace rc_controller {

PoseControllerNode::PoseControllerNode(const rclcpp::NodeOptions &options)
    : Node("pose_controller", options), offet_(0.0f) {
  RCLCPP_INFO(this->get_logger(), "Starting PoseController node!");

  // 获取位置在一区还是二区
  auto pos_request = this->create_client<rc_interface_msgs::srv::InitPos>(
      "/rc_decision/init_pose");

  get_desireLoc();
  init_PID();

  // 初始化位置
  current_pose_.x = 0;
  current_pose_.y = 0;
  current_pose_.yaw = 0;

  this->declare_parameter<std::string>("init_pose_topic", "/image_raw");
  std::string init_pose_topic =
      this->get_parameter("init_pose_topic").as_string();

  // 向摄像头识别节点请求
  // 这里只是用来锻炼能力的 -------------------- -
  init_pos_client_ = this->create_client<rc_interface_msgs::srv::InitPos>(
      "/rc_decision/init_pose");
  auto request = std::make_shared<rc_interface_msgs::srv::InitPos::Request>();
  request->names = std::string("init_pose request");
  auto future_result = init_pos_client_->async_send_request(request);
  // RCLCPP_INFO_STREAM(this->get_logger(),
  //                    "Request init_pose" << result->posmode);

  // 等待结果
  auto status = future_result.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    auto result = future_result.get();
    // 在这里处理结果
    uint8_t posmode = result->posmode;
    RCLCPP_INFO_STREAM(this->get_logger(), "Request init_pose" << posmode);
    // ...
  } else {
    // 请求超时或者其他错误
    // ...
  }
  // 这里只是用来锻炼能力的 --------------------

  // Fastlio
  //实时订阅Mid360发出来的当前位姿信息
  poseUpdate_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::SensorDataQoS(),
      std::bind(&PoseControllerNode::poseUpdate_callback, this,
                std::placeholders::_1));

  //发布位置的信息
  position_mode_.data = 0;
  position_mode_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("//rc/position_mode", 10);

  // 订阅rc_state_collector目标位姿信息
  poseCommand_sub_ = this->create_subscription<rc_interface_msgs::msg::Motion>(
      "/rc/desire_pose", rclcpp::SensorDataQoS(),
      std::bind(&PoseControllerNode::poseCommand_callback, this,
                std::placeholders::_1));

  // 发布电机控制，和下位机对接点
  cmd_pub_ =
      this->create_publisher<rc_interface_msgs::msg::Motion>("/cmd_vel", 10);
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
void PoseControllerNode::get_desireLoc() {

  // 距离到达的阈值

  // desire Pose 1

  this->declare_parameter<std::vector<double>>("desire_pose", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose1 =
      this->get_parameter("desire_pose").as_double_array();
  desire_pose1_.x = desire_pose1[0];
  desire_pose1_.y = desire_pose1[1];
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

  // desire Pose 4 取球点1
  this->declare_parameter<std::vector<double>>("desire_pose4", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose4 =
      this->get_parameter("desire_pose4").as_double_array();
  desire_pose4_.x = desire_pose3[0];
  desire_pose4_.y = desire_pose3[1];
  desire_pose4_.yaw = desire_pose3[2];

  // position_mode change thres
  this->declare_parameter<float>("dis_thres", 0.2);
  euclidisThres_ = this->get_parameter("dis_thres").as_double();

  //;
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
  yaw = yaw * 180.0f / 3.1415926f;

  // 只是想要红色的输出，并不是ERROR
  RCLCPP_INFO(this->get_logger(), "lidar x: %f, y: %f, yaw: %f", current_x,
              current_y, yaw);

  // 机器人区域状态切换
  // 如果机器人在状态1，那么就是1-2，运动到2去

  if (position_mode_.data == 0) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose1_.x, desire_pose1_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 还在跑第一段线段,继续跑
      return;
    }
    //小于阈值，位置模式设置成2
    else {
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      position_mode_.data = 1;
    }
  }

  // 如果机器人在状态2，那么就是2-2，运动到2下面
  else if (position_mode_.data == 1) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose2_.x, desire_pose2_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 还在跑第一段线段,继续跑
      return;
    } else {
      // 延时一秒，防止超调
      position_mode_.data = 2;
    }
  } else if (position_mode_.data == 2) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose3_.x, desire_pose3_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 还在跑第二段线段,继续跑
      return;
    } else {

      position_mode_.data = 3;
    }
  } else if (position_mode_.data == 3) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose4_.x, desire_pose4_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 还在跑第三段线段,继续跑
      return;
    } else {

      position_mode_.data = 4;
    }
  } else if (position_mode_.data == 4) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose3_.x, desire_pose3_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      return;
      // 还在跑第四段线段,继续跑
    } else {
      position_mode_.data = 3;
    }
  }
  // else{

  // position_mode_.data = 2;
  // }

  // else if (position_mode_.data == 2)
  // {
  //   if (euclidis(current_x, current_y, yaw, desire_pose3_.x,
  //   desire_pose3_.y,desire_pose3_.yaw) < euclidisThres_){
  //     // 还在跑第一段线段,继续跑
  //     position_mode_.data = 3;
  //   } else {
  //     // 已经跑到了范围内部
  //     }
  //   }

  current_pose_.x = current_x;
  current_pose_.y = current_y;
  current_pose_.yaw = yaw;
  RCLCPP_ERROR_STREAM(this->get_logger(),
                      "Publisher mode " << position_mode_.data);
  position_mode_pub_->publish(position_mode_);
}

inline double PoseControllerNode::euclidis(double x1, double x2, double x3,
                                           double y1, double y2, double y3) {
  return std::sqrt(std::pow(x1 - y1, 2) + std::pow(x2 - y2, 2) +
                   std::pow(x3 - y3, 2));
}

void PoseControllerNode::poseCommand_callback(
    const rc_interface_msgs::msg::Motion::ConstSharedPtr msg) {
  // do something with the pose
  // publish the control commands
  // call the PID controllers

  // @TODO 后续可以取消
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Input_Desired x: " << msg->cmd_vx
                                         << ", y: " << msg->cmd_vy
                                         << ", yaw: " << msg->desire_yaw);

  //现在这里效率不够高，后面考虑直接只用统一分量数据进行拷贝

  rc_interface_msgs::msg::Motion motion_msg;

  // Pose world_target_pose = target_xy_transform(msg->cmd_vx, msg->cmd_vy,
  // msg->desire_yaw);

  // RCLCPP_INFO_STREAM(this->get_logger(), "After transformationx Input: x" <<
  // world_target_pose.x << ", y"<< world_target_pose.y  );

  // 注意这里z是直接对应的yaw轴的转定角度
  // twist_msg.linear.x = x_controller_->pidCalculate(0.0, world_target_pose.x);


  // static float thisx,lastx,llastx,n,maxn;
  // 两帧雷达数据之间的进行线性插值插值
 
  if ((current_pose_.x - x_pose_) < inte_thres_) {
    n_ = 0;
    x_last_pose_ = current_pose_.x;
  }
  x_pose_ = x_last_pose_ + (current_pose_.x - x_last_pose_) * n_ / maxn_;

  motion_msg.cmd_vx = x_controller_->pidCalculate(x_pose_, msg->cmd_vx);


  if ((current_pose_.y - y_pose_) < inte_thres_) {
    n_ = 0;
    y_last_pose_ = current_pose_.y;
  }
  y_pose_ = y_last_pose_ + (current_pose_.y - y_last_pose_) * n_ / maxn_;

  n_ = n_ + 1;

  motion_msg.cmd_vy = y_controller_->pidCalculate(y_pose_, msg->cmd_vy);


  // 臂的调试
  // motion_msg.cmd_vx = 0;
  // twist_msg.linear.y = y_controller_->pidCalculate(0.0, world_target_pose.y);


  // 臂的调试
  // motion_msg.cmd_vy = 0;

  // desire_yaw，目标值
  motion_msg.desire_yaw = msg->desire_yaw;

  // 臂的调试
  // motion_msg.desire_yaw = 0;

  RCLCPP_WARN_STREAM(this->get_logger(), "fuck you" << motion_msg.desire_yaw);

  // measure_yaw,测量到的角度
  motion_msg.measure_yaw = current_pose_.yaw;

  motion_msg.ball_x = msg->ball_x;
  motion_msg.ball_y = msg->ball_y;

  // @TODO 后续可以取消
  RCLCPP_WARN_STREAM(this->get_logger(),
                     "Output_of_v x: "
                         << motion_msg.cmd_vx << ", y: " << motion_msg.cmd_vy
                         << ", desire_yaw: " << motion_msg.desire_yaw
                         << ", measure_yaw: " << motion_msg.measure_yaw);

  cmd_pub_->publish(motion_msg);
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

// 用于转换到世界坐标系下应有的x和y的pid控制器输入值
// 由于yaw放在下位机闭环相应很快，并且没有超调，所以可以近似认为desire_yaw就是current_yaw

Pose PoseControllerNode::target_xy_transform(double desire_world_x,
                                             double desire_world_y,
                                             double desire_yaw) {

  Pose world_target_pose;

  // 由于desire_yaw的相应速度非常快，所以根本不需要考虑c current yaw
  world_target_pose.x = cos(desire_yaw) * (desire_world_x - current_pose_.x) -
                        sin(desire_yaw) * (desire_world_y - current_pose_.y);
  world_target_pose.y = -sin(desire_yaw) * (desire_world_x - current_pose_.x) +
                        cos(desire_yaw) * (desire_world_y - current_pose_.y);

  world_target_pose.yaw = desire_yaw;
  // if (world_target_pose.yaw > Pai) {
  //   world_target_pose.yaw -= 2 * Pai;
  // } else if (world_target_pose.yaw < -Pai) {
  //   world_target_pose.yaw += 2 * Pai;
  // }

  return world_target_pose;
}

} // namespace rc_controller
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_controller::PoseControllerNode)
