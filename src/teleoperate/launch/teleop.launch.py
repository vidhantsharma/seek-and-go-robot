from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            emulate_tty=True,          # allocate a pty so it can read stdin
            prefix='xterm -e',         # optional: opens it in a new xterm (remove if you don't want a new window)
            remappings=[
                ('cmd_vel', '/cmd_vel'),  # map the node's 'cmd_vel' to absolute '/cmd_vel'
            ],
            parameters=[
                {'use_sim_time': True},
                {'speed': 0.1},         # ← default linear speed
                {'turn': 0.1},          # ← default angular speed
            ],
        )
    ])
