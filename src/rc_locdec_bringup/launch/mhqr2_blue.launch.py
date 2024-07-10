import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription


sys.path.append(
    os.path.join(get_package_share_directory("rc_locdec_bringup"), "launch")
)

config = os.path.join(
    get_package_share_directory("rc_locdec_bringup"), "config", "mhqr2_blue.yaml"
)

print(f"--------------------------Node parameters file: {config}")


def generate_launch_description():

    mhq_r2 = Node(
        package="rc_state_collector",
        executable="mhqr2node",
        name="mhq_r2_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    carry_state_node = Node(
        package="rc_detector",
        executable="rc_carry_state_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "camera_port": "/dev/video0",  # Default camera port, can be overridden by launch arguments
                "roi": [
                    190,
                    250,
                    480,
                    550,
                ],  # row range(a,b),col range(c,d)The ROI to detect whether the robot is carrying the ball
                "thres": 25000,  # The threshold of the sum of the pixel values in the ROI
                "fps": 20,
            }
        ],
        arguments=["--ros-args", "--log-level", "debug"],
    )

    return LaunchDescription([
        mhq_r2, 
        carry_state_node
    ])
