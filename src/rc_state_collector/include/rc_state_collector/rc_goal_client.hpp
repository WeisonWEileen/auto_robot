#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int64.hpp"
#include <vector>

namespace rc_state_collector {

class NavToPoseClient : public rclcpp::Node {
public:
  explicit NavToPoseClient(const rclcpp::NodeOptions &options);

private:

    //goal的client端
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      client_ptr_;
  //看导航到哪个目标点
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr nav_goal_id_sub_;
    
    // @TODO ： 写到yaml文件夹
    //添加的默认的定点位置,把四元数转换为欧拉角
  nav2_msgs::action::NavigateToPose::Goal set_pose_goal(double x, double y, double z, double ox, double oy, double oz, double ow);
  void add_goal_pose();
  std::vector<nav2_msgs::action::NavigateToPose::Goal> goal_poses_; 
  
// 回调函数，当接受到新的/goal_pose_id消息时调用，将启动底盘到定点位置
    void nav_goal_callback(const std_msgs::msg::Int64::SharedPtr msg);

    //   发送一个目标点，准备运动
    void sendGoal(nav2_msgs::action::NavigateToPose::Goal goal);

}; // namespace rc_state_collector
}