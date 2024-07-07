#ifndef RC_CONTROLLER_PID_CONTROLLER_HPP_
#define RC_CONTROLLER_PID_CONTROLLER_HPP_

#define Pai 3.1415926
// 这是控制层的PID控制器，用于控制机器人的运动

// self defined ros msgs
#include "rc_interface_msgs/msg/motion.hpp"
#include "rc_interface_msgs/srv/init_pos.hpp"

#include <cmath>
// #include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

// 三个目标点的对应的阈值
#define Area_12_XThres 6.0
#define Area_22_YThres -3.5
#define Area_23_YThres 9.0

namespace rc_controller {

struct Pose {
  double x;
  double y;
  double yaw;
};

class PIDController {
public:
  PIDController(std::vector<double>);

  // double kp, double ki, double kd,double max_out, double integral_lim

  double pidCalculate(double current, double desire_value);

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

  // 储存位置信息并且做发布
  std_msgs::msg::Int32 position_mode_;
  //位置默认的发布
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_mode_pub_;

  //初始化PID嵌套类
  void init_PID();
  // 获取连续运动的阈值
  void get_desireLoc();
  // 储存的三个目标地点的目标值
  Pose desire_pose1_;
  Pose desire_pose2_;
  Pose desire_pose3_;
  Pose desire_pose4_;
  Pose desire_pose5_;

  float euclidisThres_;

  // 发布运动控制指令
  rclcpp::Publisher<rc_interface_msgs::msg::Motion>::SharedPtr cmd_pub_;
  // 转换目标位置到机器人坐标系
  Pose target_xy_transform(double desire_world_x, double desire_world_y,
                           double desire_yaw);

  // 订阅mid360现在位置
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr poseUpdate_sub_;
  void poseUpdate_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // 用欧氏距离判断是否达到了目标值
  inline double euclidis(double x1, double x2, double x3, double y1, double y2,
                         double y3);

  // 订阅决策目标位置,z直接对应yaw轴
  rclcpp::Subscription<rc_interface_msgs::msg::Motion>::SharedPtr poseCommand_sub_;
  void
  poseCommand_callback(const rc_interface_msgs::msg::Motion::ConstSharedPtr msg);

  // 记录当前的x,y,yaw位姿，mid360订阅更新
  Pose current_pose_;
  
  // 订阅上方的摄像头位置
  rclcpp::Client<rc_interface_msgs::srv::InitPos>::SharedPtr init_pos_client_;
      // 判断在一区还是二区而决定是否要进行坐标偏置
  float offet_;


  // 使用插值提高雷达的频率
  // float current_pose_ = 0.0f;
  float x_pose_ = 0.0f;
  float x_last_pose_ = 0.0f;
  // float x_llast_pose_ = 0.0f;

  float y_pose_ = 0.0f;
  float y_last_pose_ = 0.0f;

  int maxn_ = 10;
  int n_ = 0;

  // 这个还要看雷达飘多少
  // 在我现在的位置上测得最高飘0.08m左右
  float inte_thres_ = 0.08f;
};

} // namespace rc_controller

#endif