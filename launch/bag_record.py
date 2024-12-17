from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/sjtu_drone/gt_pose',
                '/sjtu_drone/cmd_vel',
                '/ugv_gt_pose',
                '/forward_velocity_controller/commands',
                '/cable_length',
                '/target_position_uav',
                '/target_position_ugv',
                '/tether_positions',
                '/rs_robot/velodyne_plugin/out',
                '/sjtu_drone/velodyne_plugin/out'
            ],
            output='screen'
        ),
    ])
