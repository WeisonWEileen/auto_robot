import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pose_controller_node = Node(
        package="rc_controller",
        executable="posecontrollerNode",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "x_controller_":[0.0,0.0,0.0,0.0,0.0],
            "y_controller_":[0.0,0.0,0.0,0.0,0.0],
            "yaw_controller_":[0.0,0.0,0.0,0.0,0.0],
            }],
        arguments=['--ros-args','--log-level','debug'],
    )

    ld = LaunchDescription()
    ld.add_action(pose_controller_node)


    return ld