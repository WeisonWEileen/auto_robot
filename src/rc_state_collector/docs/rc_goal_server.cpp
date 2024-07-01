// pid_control_server.cpp
#include "rc_state_collector/rc_goal_server.hpp"
namespace rc_state_collector{
NavToPoseServer::NavToPoseServer(const rclcpp::NodeOptions &options)
    : Node("pid_control_server", options)
{

  // init_PID();
  
  action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "navigate_to_pose",
      std::bind(&NavToPoseServer::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&NavToPoseServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavToPoseServer::handle_accepted, this,
                std::placeholders::_1));
  pose_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
          "/Odometry", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          { this->cur_pose_ = msg; });

  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&NavToPoseServer::control_loop, this));
} // namespace
  // NavToPoseServer::NavToPoseServer(constrclcpp::NodeOptions&options):Node("pid_control_server",options),pid_x_(1.0,0.0,0.0),pid_y_(1.0,0.0,0.0)

rclcpp_action::GoalResponse NavToPoseServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with target position (%.2f, %.2f)",
              goal->pose.pose.position.x, goal->pose.pose.position.y);
  target_pose_ = goal->pose;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavToPoseServer::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavToPoseServer::handle_accepted(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
  std::thread{
      std::bind(&NavToPoseServer::execute, this, std::placeholders::_1),
      goal_handle}
      .detach();
}

void NavToPoseServer::execute(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
  rclcpp::Rate loop_rate(10);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto &distance = feedback->distance_remaining;
  auto result = std::make_shared<NavigateToPose::Result>();

  while (rclcpp::ok()) {
    double current_x =
        get_current_x_position(); // 你需要实现这个函数来获取当前x位置
    double current_y =
        get_current_y_position(); // 你需要实现这个函数来获取当前y位置
    double target_x = target_pose_.pose.position.x;
    double target_y = target_pose_.pose.position.y;

    double dx = target_x - current_x;
    double dy = target_y - current_y;
    distance = sqrt(dx * dx + dy * dy);

    // 如果距离小于0.05米，认为到达目标点
    // @TODO这里还不行，需要根据实际情况调整
    // if (distance < RADIUS_TOLERANCE) {
    //   result->result = true;
    //   goal_handle->succeed(result);
    //   RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    //   return;
    // }

    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }
}

void NavToPoseServer::control_loop() {
  static auto last_time = this->now();
  auto current_time = this->now();
  last_time = current_time;

  pid_x_.measure_  = get_current_x_position(); // 你需要实现这个函数来获取当前x位置
  pid_y_.measure_ = get_current_y_position(); // 你需要实现这个函数来获取当前y位置



  double target_x = target_pose_.pose.position.x;
  double target_y = target_pose_.pose.position.y;

  double control_x = pid_x_.pidCalculate(target_x);
  double control_y = pid_y_.pidCalculate(target_y);

  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.x = control_x;
  cmd_vel.linear.y = control_y; // 仅在全向机器人时使用
  cmd_vel_publisher_->publish(cmd_vel);
}

double NavToPoseServer::get_current_x_position() {
  // TODO: 实现获取当前x位置的方法

  return this->cur_pose_->pose.pose.position.x;
}

double NavToPoseServer::get_current_y_position() {
  // TODO: 实现获取当前y位置的方法
  return this->cur_pose_->pose.pose.position.y;
}
  // void NavToPoseServer::init_PID() {
  //   this->declare_parameter<std::vector<double>>("x_controller_",
  //                                                {0.0, 0.0, 0.0, 0.0, 0.0});
  //   this->declare_parameter<std::vector<double>>("y_controller_",
  //                                                {0.0, 0.0, 0.0, 0.0, 0.0});
  //   this->declare_parameter<std::vector<double>>("yaw_controller_",
  //                                                {0.0, 0.0, 0.0, 0.0, 0.0});

  //   std::vector<double> x_pid_param =
  //       this->get_parameter("x_controller_").as_double_array();
  //   std::vector<double> y_pid_param =
  //       this->get_parameter("y_controller_").as_double_array();
  //   std::vector<double> yaw_pid_param =
  //       this->get_parameter("z_controller_").as_double_array();

  //   // @TODO yaml添加pid初始化结构体的struct定义
  //   x_controller_ = std::make_unique<rc_controller::PIDController>(x_pid_param);
  //   y_controller_ = std::make_unique<rc_controller::PIDController>(y_pid_param);
  //   yaw_controller_ =
  //       std::make_unique<rc_controller::PIDController>(yaw_pid_param);
  // }
}


// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(rc_state_collector::NavToPoseServer)

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<NavToPoseServer>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

// }