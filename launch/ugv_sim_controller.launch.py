#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'theatre.model'   

    world = os.path.join(get_package_share_directory('marsupial_simulator_ros2'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_marsupial_simulator_ros2 = get_package_share_directory('marsupial_simulator_ros2')

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )

    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        )

    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_marsupial_simulator_ros2, 'launch', 'marsupial_simulator_ros2.launch.py')
            ),
        )

    ugv_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ugv_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'], output='screen'
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
        )

    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
        )
        
    spawn_ugv_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'rs_robot', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'rs_robot/rs_robot.sdf'), '-x', '0', '-y', '0', '-z', '2'],
            output='screen',
        )   
     
    joy_node = Node(
        package = "joy",
        executable = "joy_node"
        )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    nodes = [
        gzserver,
        gzclient, 
        ugv_state_publisher,
        spawn_ugv_node,
        joy_node,
        controller,
        forward_position_controller,
        forward_velocity_controller,
        joint_state_broadcaster,
    ]

    return LaunchDescription(nodes)

