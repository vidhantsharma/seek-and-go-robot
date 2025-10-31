# launch/lidar_slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from datetime import datetime
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Assume workspace is one directory above 'src'
    launch_dir = os.path.abspath(os.path.dirname(__file__))
    workspace_root = os.path.abspath(os.path.join(launch_dir, '..', '..', '..'))
    slam_dir = os.path.join(workspace_root, 'slam_maps', datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(slam_dir, exist_ok=True)

    # Log the path so it appears in the launch output
    log_slam_dir = LogInfo(msg=['SLAM directory: ', slam_dir])

    # Path to your YAML config file
    slam_yaml = os.path.join(
        get_package_share_directory('slam'),
        'config',
        'slam_toolbox.yaml'
    )

    # Start SLAM toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', # or 'sync_slam_toolbox_node' / 'async_slam_toolbox_node'
        name='slam_toolbox',
        output='screen',
        # arguments=['--ros-args','--log-level','debug'],
        parameters=[slam_yaml]  # Load YAML into parameter server
    )

    # Start map saver node
    save_map_node = Node(
        package='slam',
        executable='save_map_on_demand.py',
        name='map_saver_wrapper',
        output='screen',
        parameters=[{'save_folder': slam_dir}]
    )

    return LaunchDescription([log_slam_dir, slam_node, save_map_node])

