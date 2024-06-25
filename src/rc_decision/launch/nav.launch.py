from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rc_decision',
            executable='rc_nav',
            name='rc_nav',
            output='screen',
        ),
        Node(
            package='rc_decision',
            executable='read_nav_status',
            name='read_nav_status',
            output='screen',
        ),
        Node(
            package='rc_decision',
            executable='rc_move',
            name='rc_move',
            output='screen',
        ),
        Node(
            package='rc_decision',
            executable='tf_listener',
            name='tf_listener',
            output='screen',
        ),
    ])