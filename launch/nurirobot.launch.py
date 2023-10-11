from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2-nurirobot-driver',
            executable='main',
            name='nurirobot_driver_node',
            output='screen',
            emulate_tty=True,
        )
    ])

# namespace='nurirobot_driver_node',
            