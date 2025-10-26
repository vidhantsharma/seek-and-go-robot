from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',  # opens a new terminal window for keyboard input (optional)
            remappings=[
                ('/cmd_vel', '/cmd_vel'),  # keep same topic name
            ],
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])
