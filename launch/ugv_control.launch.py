from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(package='marsupial_simulator_ros2', 
            executable='ugv_control_controller.py', 
            # executable='ugv_control_keyboard.py', 
            output='screen',
            # namespace='ugv'
        ),
    ])