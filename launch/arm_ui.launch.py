from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rk_demo',
            executable='arm_ui',
            name='rk_arm_ui',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}]
        )
    ])
