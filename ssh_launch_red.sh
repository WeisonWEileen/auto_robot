#!/bin/bash

ros2 launch fast_lio mapping.launch.py 
ros2 launch livox_ros_driver2 msg_MID360_launch.py &&
ros2 launch rc_locdec_bringup locdec_bingup.launch.py &&
ros2 launch rc_serial_driver serial_driver.launch.py &&
ros2 launch rc_vision_bringup vision_bringup.launch.py