colcon build --symlink-install
cmds=(
    "ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=RC2024 \
    mode:=nav \
    lio:=fastlio \
    localization:=icp \
    lio_rviz:=False \
    nav_rviz:=False"
    # "python3 /home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/test.py"
    "ros2 launch serial_driver command.launch.py"
    "ros2 launch rc_decision nav.launch.py"
    "ros2 launch rc_decision decision.launch.py"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.5
done

