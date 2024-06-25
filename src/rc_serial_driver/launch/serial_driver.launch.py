import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([rc_serial_driver_node])
