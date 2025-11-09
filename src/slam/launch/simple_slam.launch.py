from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam',
            executable='simple_slam_node',
            name='simple_2d_slam',
            output='screen',
            parameters=[{
                # tune these if needed
                'map_width_m': 40.0,
                'map_height_m': 40.0,
                'map_resolution': 0.05,
                'scan_downsample': 2,
                'icp_max_iterations': 25,
                'icp_max_correspondence_dist': 0.5,
                'icp_convergence_tol': 1e-4,
                'odom_use': True,
                'use_sim_time': True
            }]
        )
    ])
