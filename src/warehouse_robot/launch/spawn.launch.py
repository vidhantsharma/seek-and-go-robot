# spawn.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare(package='warehouse_robot')

    # world file resolved relative to package share
    world = PathJoinSubstitution([pkg_share, 'worlds', 'warehouse.world'])

    # start gazebo classic with world
    gz = ExecuteProcess(
        cmd=[FindExecutable(name='gazebo'), '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
        shell=False
    )

    # path to xacro inside package
    xacro_file = PathJoinSubstitution([pkg_share, 'models', 'urdf', 'warehouse_robot.xacro'])

    # path to meshes directory (absolute path substitution will be provided to xacro)
    meshes_dir = PathJoinSubstitution([pkg_share, 'models', 'meshes'])

    # Build the xacro command and pass meshes_dir:=<absolute path>
    # Note: spaces in the list are intentional so the substitution builds a single command string.
    robot_description_command = Command([
        FindExecutable(name='xacro'), ' ', xacro_file, ' ', 'meshes_dir:=', meshes_dir
    ])

    # wrap as string so ROS2 doesn't try to parse YAML
    robot_description_param = {'robot_description': ParameterValue(robot_description_command, value_type=str)}

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'warehouse_robot']
    )

    return LaunchDescription([gz, robot_state_pub, spawn])
