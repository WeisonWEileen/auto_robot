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
    get_package_share_directory("rc_locdec_bringup"), "config", "mhqr2_red.yaml"
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

    return LaunchDescription([mhq_r2])
