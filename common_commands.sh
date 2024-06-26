#!/bin/bash
echo "常用指令"
echo "ros2 topic echo /Odometry | grep pose -A4 - 查看位置"
echo "ros2 run rqt_robot_steering  rqt_robot_steering - 可视化修改/cmd_vel"
echo "ros2 run tf2_tools view_frames - 查看tf"
echo "colcon build --symlink-install --packages-ignore serial_driver - launch需要  "