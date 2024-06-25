from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rc_decision',
            executable='game_start',
            name='game_start',
            output='screen',
        ),
        Node(
            package='rc_decision',
            executable='master_decision_32',
            name='master_decision_32',
            output='screen',
        ),
        Node(
            package='rc_decision',
            executable='send_chioce',
            name='send_chioce',
            output='screen',
        ),
        Node(
            package='rc_decision',
            executable='vision_decision',
            name='vision_decision',
            output='screen',
        ),
    ])