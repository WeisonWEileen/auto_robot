#include "rc_controller/pid_controller.hpp"
#include <chrono>
// #include

#define Area_12_XThres 6.0
#define Area_22_YThres -3.5
#define Area_23_XThres 9.0

// 缁堢鏃ュ織
// 绾㈣壊锛氬搴旂殑鏄浄杈句綅缃洿鏂颁俊锟??
// 榛勮壊锛氬搴旂殑鏄疨ID鎺у埗鍣ㄧ殑杈撳嚭
// 鐏拌壊锛氬搴旂殑鏄痳c_state_collector缁欑殑鍛戒护

namespace rc_controller {

PoseControllerNode::PoseControllerNode(const rclcpp::NodeOptions &options)
    : Node("pose_controller", options), offet_(0.0f) {
  RCLCPP_INFO(this->get_logger(), "Starting PoseController node!");

  // 鑾峰彇浣嶇疆鍦ㄤ竴鍖鸿繕鏄簩锟??
  auto pos_request = this->create_client<rc_interface_msgs::srv::InitPos>(
      "/rc_decision/init_pose");

  get_desireLoc();
  init_PID();

  // 鍒濆鍖栦綅锟??
  current_pose_.x = 0;
  current_pose_.y = 0;
  current_pose_.yaw = 0;

  this->declare_parameter<std::string>("init_pose_topic", "/image_raw");
  std::string init_pose_topic = this->get_parameter("init_pose_topic").as_string();

  // 鍚戞憚鍍忓ご璇嗗埆鑺傜偣璇锋眰
  // 杩欓噷鍙槸鐢ㄦ潵閿荤偧鑳藉姏锟?? -------------------- -
  init_pos_client_ = this->create_client<rc_interface_msgs::srv::InitPos>(
      "/rc_decision/init_pose");
  auto request = std::make_shared<rc_interface_msgs::srv::InitPos::Request>();
  request->names = std::string("init_pose request");
  auto future_result = init_pos_client_->async_send_request(request);
  // RCLCPP_INFO_STREAM(this->get_logger(),
  //                    "Request init_pose" << result->posmode);

  // 绛夊緟缁撴灉
  auto status = future_result.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    auto result = future_result.get();
    // 鍦ㄨ繖閲屽鐞嗙粨锟??
    uint8_t posmode = result->posmode;
    RCLCPP_INFO_STREAM(this->get_logger(), "Request init_pose" << posmode);
    // ...
  } else {
    // 璇锋眰瓒呮椂鎴栬€呭叾浠栭敊锟??
    // ...
  }
  // 杩欓噷鍙槸鐢ㄦ潵閿荤偧鑳藉姏锟?? --------------------

  // Fastlio
  //瀹炴椂璁㈤槄Mid360鍙戝嚭鏉ョ殑褰撳墠浣嶅Э淇℃伅
  poseUpdate_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::SensorDataQoS(),
      std::bind(&PoseControllerNode::poseUpdate_callback, this,
                std::placeholders::_1));

  //鍙戝竷浣嶇疆鐨勪俊锟??
  position_mode_.data = 0;
  position_mode_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("/rc/position_mode", 10);

  // 璁㈤槄rc_state_collector鐩爣浣嶅Э淇℃伅
  poseCommand_sub_ = this->create_subscription<rc_interface_msgs::msg::Motion>(
      "/rc/desire_pose", rclcpp::SensorDataQoS(),
      std::bind(&PoseControllerNode::poseCommand_callback, this,
                std::placeholders::_1));

  // 鍙戝竷鐢垫満鎺у埗锛屽拰涓嬩綅鏈哄鎺ョ偣
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

  // @TODO yaml娣诲姞pid鍒濆鍖栫粨鏋勪綋鐨剆truct瀹氫箟
  x_controller_ = std::make_unique<PIDController>(x_pid_param);
  y_controller_ = std::make_unique<PIDController>(y_pid_param);
  yaw_controller_ = std::make_unique<PIDController>(yaw_pid_param);
}

//
void PoseControllerNode::get_desireLoc() {

  // 璺濈鍒拌揪鐨勯槇锟??

  // desire Pose 1

  this->declare_parameter<std::vector<double>>("desire_pose1", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose1 =
      this->get_parameter("desire_pose1").as_double_array();
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

  // desire Pose 4 鍙栫悆锟??1
  this->declare_parameter<std::vector<double>>("desire_pose4", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose4 =
      this->get_parameter("desire_pose4").as_double_array();
  desire_pose4_.x = desire_pose4[0];
  desire_pose4_.y = desire_pose4[1];
  desire_pose4_.yaw = desire_pose4[2];

    this->declare_parameter<std::vector<double>>("desire_pose5", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose5 =
      this->get_parameter("desire_pose5").as_double_array();
  desire_pose5_.x = desire_pose5[0];
  desire_pose5_.y = desire_pose5[1];
  desire_pose5_.yaw = desire_pose5[2];

      this->declare_parameter<std::vector<double>>("desire_pose6", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose6 =
      this->get_parameter("desire_pose6").as_double_array();
  desire_pose6_.x = desire_pose6[0];
  desire_pose6_.y = desire_pose6[1];
  desire_pose6_.yaw = desire_pose6[2];

  this->declare_parameter<std::vector<double>>("desire_pose7", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose7 =
      this->get_parameter("desire_pose7").as_double_array();
  desire_pose7_.x = desire_pose7[0];
  desire_pose7_.y = desire_pose7[1];
  desire_pose7_.yaw = desire_pose7[2];

   this->declare_parameter<std::vector<double>>("desire_pose8", {0.0, 0.0, 0.0});
  std::vector<double> desire_pose8 =
      this->get_parameter("desire_pose8").as_double_array();
  desire_pose8_.x = desire_pose8[0];
  desire_pose8_.y = desire_pose8[1];
  desire_pose8_.yaw = desire_pose8[2];


  // position_mode change thres
  this->declare_parameter<float>("dis_thres", 0.2);
  euclidisThres_ = this->get_parameter("dis_thres").as_double();

  //;
}

// 璁㈤槄mid360鐨勯┍鍔ㄦ帴锟??
void PoseControllerNode::poseUpdate_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  // 鍏堢洿鎺ヨ幏鍙杧鍜寉
  auto current_x = msg->pose.pose.position.x;
  auto current_y = msg->pose.pose.position.y;

  //璁㈤槄鍥涘厓鏁扮殑瑙掑害锛岃浆鎹负yaw杞寸殑瑙掑害
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = yaw * 180.0f / 3.1415926f;

  // 鍙槸鎯宠绾㈣壊鐨勮緭鍑猴紝骞朵笉鏄疎RROR
  RCLCPP_INFO(this->get_logger(), "lidar x: %f, y: %f, yaw: %f", current_x,
              current_y, yaw);

  // 鏈哄櫒浜哄尯鍩熺姸鎬佸垏锟??
  // 濡傛灉鏈哄櫒浜哄湪鐘讹拷?1锛岄偅涔堝氨锟??1-2锛岃繍鍔ㄥ埌2锟??

  if (position_mode_.data == 0) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose1_.x, desire_pose1_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠涓€娈电嚎锟??,缁х画锟??
      // return;
    }
    //灏忎簬闃堝€硷紝浣嶇疆妯″紡璁剧疆锟??2
    else {
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      position_mode_.data = 1;
    }
  }

  // 濡傛灉鏈哄櫒浜哄湪鐘讹拷?2锛岄偅涔堝氨锟??2-2锛岃繍鍔ㄥ埌2涓嬮潰
  else if (position_mode_.data == 1) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose2_.x, desire_pose2_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠涓€娈电嚎锟??,缁х画锟??
      // return;
    } else {
      // 寤舵椂涓€绉掞紝闃叉瓒呰皟
      position_mode_.data = 2;
    }
  } else if (position_mode_.data == 2) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose3_.x, desire_pose3_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠浜屾绾挎,缁х画锟??
    } else {

      position_mode_.data = 3;
    }
  } else if (position_mode_.data == 3) {
    double current_thres =
        euclidis(current_x, current_y, 0, desire_pose4_.x, desire_pose4_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠涓夋绾挎,缁х画锟??
    } else {
      // 鍒拌揪浜嗙鍥涗釜鐐逛簡
      position_mode_.data = 4;
    }
  } else if (position_mode_.data == 4) {
    
        double current_thres =
        euclidis(current_x, current_y, 0, desire_pose5_.x, desire_pose5_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠鍥涙绾挎,缁х画锟??

    } else {
      position_mode_.data = 5;
    }
    }

    else if (position_mode_.data == 5) {
      double current_thres = euclidis(current_x, current_y, 0, desire_pose6_.x, desire_pose6_.y, 0);
      RCLCPP_ERROR_STREAM(this->get_logger(),"current thres is " << current_thres);
      if (current_thres > euclidisThres_) {
      } else {
        position_mode_.data = 6;
      }
    }
    else if (position_mode_.data == 6) {
    
        double current_thres =
        euclidis(current_x, current_y, 0, desire_pose7_.x, desire_pose7_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠鍥涙绾挎,缁х画锟??

    } else {
      position_mode_.data = 7;
    }
  } else if (position_mode_.data == 7) {
        double current_thres =
        euclidis(current_x, current_y, 0, desire_pose8_.x, desire_pose8_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠鍥涙绾挎,缁х画锟??

    } else {
      position_mode_.data = 8;
    }
  }
  else if (position_mode_.data == 8) {
        double current_thres =
        euclidis(current_x, current_y, 0, desire_pose7_.x, desire_pose7_.y, 0);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "current thres is " << current_thres);
    if (current_thres > euclidisThres_) {
      // 杩樺湪璺戠鍥涙绾挎,缁х画锟??

    } else {
      position_mode_.data = 7;
    }
  }

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

  // @TODO 鍚庣画鍙互鍙栨秷
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Input_Desired x: " << msg->cmd_vx
                                         << ", y: " << msg->cmd_vy
                                         << ", yaw: " << msg->desire_yaw);

  //鐜板湪杩欓噷鏁堢巼涓嶅楂橈紝鍚庨潰鑰冭檻鐩存帴鍙敤缁熶竴鍒嗛噺鏁版嵁杩涜鎷疯礉

  rc_interface_msgs::msg::Motion motion_msg;

 
  // if ((current_pose_.x - x_pose_) < inte_thres_) {
    // n_ = 0;
    // x_last_pose_ = current_pose_.x;
  // }
  // x_pose_ = x_last_pose_ + (current_pose_.x - x_last_pose_) * n_ / maxn_;

  // motion_msg.cmd_vx = x_controller_->pidCalculate(x_pose_, msg->cmd_vx);
  motion_msg.cmd_vx = x_controller_->pidCalculate(current_pose_.x, msg->cmd_vx);

  // if ((current_pose_.y - y_pose_) < inte_thres_) {
    // n_ = 0;
    // y_last_pose_ = current_pose_.y;
  // }

  // y_pose_ = y_last_pose_ + (current_pose_.y - y_last_pose_) * n_ / maxn_;

  // n_ = n_ + 1;

  // motion_msg.cmd_vy = y_controller_->pidCalculate(y_pose_, msg->cmd_vy);
  motion_msg.cmd_vy = y_controller_->pidCalculate(current_pose_.y, msg->cmd_vy);

  // 鑷傜殑璋冭瘯
  // motion_msg.cmd_vx = 0;
  // twist_msg.linear.y = y_controller_->pidCalculate(0.0, world_target_pose.y);


  // 鑷傜殑璋冭瘯
  // motion_msg.cmd_vy = 0;

  // desire_yaw锛岀洰鏍囷拷?
  motion_msg.desire_yaw = msg->desire_yaw;

  // 鑷傜殑璋冭瘯
  // motion_msg.desire_yaw = 0;


  // measure_yaw,娴嬮噺鍒扮殑瑙掑害
  motion_msg.measure_yaw = current_pose_.yaw;
  RCLCPP_WARN_STREAM(this->get_logger(), "fuck you" << motion_msg.measure_yaw);

  motion_msg.ball_x = msg->ball_x;
  motion_msg.ball_y = msg->ball_y;
  motion_msg.mode = 1;

      // @TODO 鍚庣画鍙互鍙栨秷
  RCLCPP_WARN_STREAM(this->get_logger(),
                     "Output_of_v x: "
                         << motion_msg.cmd_vx << ", y: " << motion_msg.cmd_vy
                         << ", desire_yaw: " << motion_msg.desire_yaw
                         << ", measure_yaw: " << motion_msg.measure_yaw
                         << " ballx: " << msg->ball_x);

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

  // 绉垎闄愬箙
  if (Iout_ > integral_lim_) {
    Iout_ = integral_lim_;
  } else if (Iout_ < -integral_lim_) {
    Iout_ = -integral_lim_;
  }

  Dout_ = kd_ * (err_ - last_err_);
  output_ = Pout_ + Iout_ + Dout_;

  // 杈撳嚭闄愬箙
  if (output_ > maxOut_) {
    output_ = maxOut_;
  } else if (output_ < -maxOut_) {
    output_ = -maxOut_;
  }

  last_err_ = err_;
  last_dout_ = Dout_;

  return output_;
}

// 鐢ㄤ簬杞崲鍒颁笘鐣屽潗鏍囩郴涓嬪簲鏈夌殑x鍜寉鐨刾id鎺у埗鍣ㄨ緭鍏ワ拷?
// 鐢变簬yaw鏀惧湪涓嬩綅鏈洪棴鐜浉搴斿緢蹇紝骞朵笖娌℃湁瓒呰皟锛屾墍浠ュ彲浠ヨ繎浼艰涓篸esire_yaw灏辨槸current_yaw

Pose PoseControllerNode::target_xy_transform(double desire_world_x,
                                             double desire_world_y,
                                             double desire_yaw) {

  Pose world_target_pose;

  // 鐢变簬desire_yaw鐨勭浉搴旈€熷害闈炲父蹇紝鎵€浠ユ牴鏈笉闇€瑕佽€冭檻c current yaw
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
