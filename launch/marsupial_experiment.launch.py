from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    uav_x = LaunchConfiguration('uav_x', default='0.0')
    uav_y = LaunchConfiguration('uav_y', default='0.0')
    uav_z = LaunchConfiguration('uav_z', default='1.0')
    ugv_x = LaunchConfiguration('ugv_x', default='0.0')
    ugv_y = LaunchConfiguration('ugv_y', default='0.0')

    def pub_initial_positions(context):
        return [
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '/target_position_uav', 'geometry_msgs/msg/Pose',
                     '{position: {x: ' + str(context.launch_configurations['uav_x']) + 
                     ', y: ' + str(context.launch_configurations['uav_y']) + 
                     ', z: ' + str(context.launch_configurations['uav_z']) + 
                     '}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}', '--once'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '/target_position_ugv', 'geometry_msgs/msg/Pose',
                     '{position: {x: ' + str(context.launch_configurations['ugv_x']) + 
                     ', y: ' + str(context.launch_configurations['ugv_y']) + 
                     ', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}', '--once'],
                output='screen'
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('uav_x', default_value='0.0', description='Initial X position for UAV'),
        DeclareLaunchArgument('uav_y', default_value='0.0', description='Initial Y position for UAV'),
        DeclareLaunchArgument('uav_z', default_value='1.0', description='Initial Z position for UAV'),
        DeclareLaunchArgument('ugv_x', default_value='0.0', description='Initial X position for UGV'),
        DeclareLaunchArgument('ugv_y', default_value='0.0', description='Initial Y position for UGV'),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/sjtu_drone/takeoff', 'std_msgs/msg/Empty', '{}', '--once'],
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                OpaqueFunction(function=pub_initial_positions)
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='marsupial_simulator_ros2',
                    executable='trajectory_follower.py',
                    name='trajectory_follower',
                    output='screen',
                    arguments=[LaunchConfiguration('mission')]
                )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='marsupial_simulator_ros2', 
                    executable='ugv_theter_trajectory_follower.py',
                    name='ugv_theter_to_point',
                    output='screen'
                ),
                Node(
                    package='marsupial_simulator_ros2',  
                    executable='uav_trajectory_follower.py',
                    name='uav_to_point',
                    output='screen'
                ),
            ]
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', 
                 '/sjtu_drone/gt_pose', 
                 '/sjtu_drone/cmd_vel', 
                 '/ugv_gt_pose', 
                 '/forward_velocity_controller/commands', 
                 '/cable_length', 
                 '/target_position_uav', 
                 '/target_position_ugv',
                 '/tether_positions'],
            output='screen'
        ),
    ])
