#include "rc_state_collector/rc_goal_client.hpp"
namespace rc_state_collector {

NavToPoseClient::NavToPoseClient(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("rc_nav_client", options) {
  client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "/navigate_to_pose");

    // 创建一个订阅器以接收/nav_goal_id话题
  this->nav_goal_id_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "/nav_goal_id", 10,
      std::bind(&NavToPoseClient::nav_goal_callback, this,
                std::placeholders::_1));


      
}

nav2_msgs::action::NavigateToPose::Goal  NavToPoseClient::set_pose_goal(
    double x, double y, double z, double ox, double oy, double oz, double ow) {
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = z;
  goal.pose.pose.orientation.x = ox;
  goal.pose.pose.orientation.y = oy;
  goal.pose.pose.orientation.z = oz;
  goal.pose.pose.orientation.w = ow;
  return goal;
}

// @TODO 加上多个目标ID点，红蓝方条件编译
void NavToPoseClient::add_goal_pose() {
  goal_poses_.push_back(
      // 设置一个x轴方向运动1m的命令
      set_pose_goal(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
}

// 发送一个目标点，准备运动
void NavToPoseClient::nav_goal_callback(
    const std_msgs::msg::Int64::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "接收到目标点id: '%ld'", msg->data);

  // 根据接收到的nav_goal_id从数组中选择目标点
  if (msg->data >= 0 &&
      static_cast<size_t>(msg->data) < this->goal_poses_.size()) {
    this->sendGoal(this->goal_poses_[msg->data]);
  } else {
    RCLCPP_ERROR(this->get_logger(), "无效的目标点id: '%ld'", msg->data);
  }
}

void NavToPoseClient::sendGoal(nav2_msgs::action::NavigateToPose::Goal goal){
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  this->client_ptr_->async_send_goal(goal, send_goal_options);
  
  // 添加一个回调函数处理成功和失败的情况，@TODO，这里要细究
  send_goal_options.goal_response_callback =
      [this](std::shared_ptr<
             rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "目标发送失败");
        } else {
          RCLCPP_INFO(this->get_logger(), "目标发送成功");
        }
      };
}

} // namespace rc_state_collector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_state_collector::NavToPoseClient)