import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    os.path.join(get_package_share_directory("rc_locdec_bringup"), "launch")
)

node_params = os.path.join(
    get_package_share_directory("rc_locdec_bringup"), "config", "node_param_red.yaml"
)

print(f"--------------------------Node parameters file: {node_params}")

def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_composable_node(package, plugin, name):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    def get_container_node(*regestered_nodes):
        return ComposableNodeContainer(
            name="locdec_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=list(regestered_nodes),
            output="both",
            emulate_tty=True,
            parameters=[node_params],
            ros_arguments=[
                "--ros-args",
                "--log-level","info"
            ],
            on_exit=Shutdown(),
        )

    # --------------------------------------#
    # --------composable_node part----------#

    controller_node = get_composable_node(
        "rc_controller", "rc_controller::PoseControllerNode", "controller_node"
    )

    state_collector_node = get_composable_node(
        "rc_state_collector",
        "rc_state_collector::StateCollectorNode",
        "state_collector_node",
    )

    # 总的接口
    locdec_node = get_container_node(
        controller_node,
        state_collector_node,
    )

    # -----------------------------#
    # --------delay part-----------#

    # delay_serial_node = TimerAction(
    #     period=1.5,
    #     actions=[serial_driver_node],
    # )

    # delay_tracker_node = TimerAction(
    #     period=2.0,
    #     actions=[tracker_node],
    # )

    # --------delay part-----------#
    # -----------------------------#

    # ------------------------------#
    # --------serial driver---------#
    config = os.path.join(
        get_package_share_directory('rc_serial_driver'), 'config', 'serial_driver.yaml')

    rc_serial_driver_node = Node(
        package='rc_serial_driver',
        executable='rc_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )
    # --------serial driver---------#
    # ------------------------------#

    return LaunchDescription(
        [
            # realsense_launch,
            locdec_node,
            # rc_serial_driver_node,
        ]
    )
