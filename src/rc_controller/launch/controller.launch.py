import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pose_controller_node = Node(
        package="rc_controller",
        executable="posecontrollerNode",
        name="nnd",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                # x   :
                # y   :
                # yaw : 3000左右合适
                "x_controller_": [0.0, 0.0, 0.0, 0.0, 0.0],
                "y_controller_": [0.0, 0.0, 0.0, 0.0, 00.0],
                "yaw_controller_": [30.0, 10.0, 12.0, 5000.0, 3000.0],
                "scale_factor_": 3.0,
            }
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    state_collector_node = Node(
        package="rc_state_collector",
        executable="statecollectorNode",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                # 注意第三个是以pi为单位的
                "desirce_pose": [0.0, 0.0,0.5],
                "scale_factor": 1.0
                # "y_controller_": [1.0, 0.0, 0.0, 0.0, 10.0],
                # "yaw_controller_": [0.0, 0.0, 0.0, 0.0, 0.0],
            }
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ld = LaunchDescription()
    ld.add_action(pose_controller_node)
    # ld.add_action(state_collector_node)

    return ld
