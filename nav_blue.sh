conda deactivate
#colcon build --symlink-install
source install/setup.bash

cmds=(
    "ros2 launch fast_lio mapping.launch.py"
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    "ros2 launch rc_locdec_bringup locdec_bringup_red.launch.py"
    "ros2 launch rc_serial_driver serial_driver.launch.py"
    "ros2 launch rc_vision_bringup vision_bringup.launch.py"
)


for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.5
done

