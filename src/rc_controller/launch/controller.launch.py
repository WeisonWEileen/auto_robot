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
                #   kp_ = pid_param[0];
                #   ki_ = pid_param[1];
                #   kd_ = pid_param[2];
                #   maxOut_ = pid_param[3];
                #   integral_lim_ = pid_param[4];
                # x   :
                # y   :
                # yaw : 3000左右合适
                # "x_controller_": [1300.0, 0.0, 230.0, 4000.0, 4000.0],
                "x_controller_": [1300.0, 0.0, 220.0, 0.0, 0.0],
                # "y_controller_": [0.0, 0.0, 0.0, 0.0, 00.0],
                "y_controller_": [1000.0, 0.1, 220.0, 3000.0, 350.0],
                "yaw_controller_": [200.0, 5.0, 4, 3000.0, 2000.0],
                # "yaw_controller_": [0.0, 5.0, 4, 0.0, 0.0],
            }
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # state_collector_node = Node(
    #     package="rc_state_collector",
    #     executable="statecollectorNode",
    #     namespace="",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[
    #         {
    #             # 注意第三个是以pi为单位的
    #             "desirce_pose": [0.0, 0.0,0.5],
    #             "scale_factor": 1.0
    #             # "y_controller_": [1.0, 0.0, 0.0, 0.0, 10.0],
    #             # "yaw_controller_": [0.0, 0.0, 0.0, 0.0, 0.0],
    #         }
    #     ],
    #     arguments=["--ros-args", "--log-level", "info"],
    # )

    ld = LaunchDescription()
    ld.add_action(pose_controller_node)
    # ld.add_action(state_collector_node)

    return ld
