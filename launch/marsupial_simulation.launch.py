#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

#####   To create new tether model modify parameters on marsupial_simulator_ros2/scripts/jinja_gen.py    #####
##### python3 /home/upo/marsupial/src/marsupial_simulator_ros2/scripts/jinja_gen.py /home/upo/marsupial/src/marsupial_simulator_ros2/models/tether/tether.sdf.jinja /home/upo/marsupial/src/marsupial_simulator_ros2/models/tether    

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = LaunchConfiguration('world', default='theatre.world')
    world = [os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'worlds/'), world_file_name]
    launch_file_dir = os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_marsupial_simulator_ros2 = get_package_share_directory('marsupial_simulator_ros2')
    ns_drone = "sjtu_drone"

    # Initial position
    init_pos_x = LaunchConfiguration('pos_x', default='0.0')
    init_pos_y = LaunchConfiguration('pos_y', default='0.0')
    init_pos_z = LaunchConfiguration('pos_z', default='0.0')
    init_pos_z = PythonExpression([init_pos_z, ' + 0.3'])

    uav_pos_x = PythonExpression([init_pos_x, ' + 0.2'])
    uav_pos_y = PythonExpression([init_pos_y, ' + 0.0'])
    uav_pos_z = PythonExpression([init_pos_z, ' + 0.4'])

    tether_pos_x = PythonExpression([init_pos_x, ' - 0.25'])
    tether_pos_y = PythonExpression([init_pos_y, ' - 0.0'])
    tether_pos_z = PythonExpression([init_pos_z, ' + 0.325'])

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
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg_marsupial_simulator_ros2, 'models/rs_robot/config', 'controllers.yaml')],
        output='screen'
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
            arguments=['-entity', 'rs_robot', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'rs_robot/rs_robot.sdf'), 
                       '-x', init_pos_x, '-y', init_pos_y, '-z', init_pos_z],
            output='screen',
        )   
    
    spawn_uav_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'sjtu_drone', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'sjtu_drone/sjtu_drone.sdf'), 
                       '-x', uav_pos_x, '-y', uav_pos_y, '-z', uav_pos_z],
            output='screen',
        )  
    
    spawn_tether_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'tether', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'tether/tether.sdf'), 
                       '-x', tether_pos_x, '-y', tether_pos_y, '-z', tether_pos_z],
            output='screen',
        ) 
    
    attach_tether_node = ExecuteProcess(
        cmd=['ros2', 'run', 'marsupial_simulator_ros2', 'attach_tether.py'],
        output='screen'
    )

    spawn_theatre = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'teatro', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'teatro/teatro.sdf'), 
                       '-x', init_pos_x, '-y', init_pos_y, '-z', init_pos_z],
            output='screen',
        ) 

    delayed_spawn_tether_node = TimerAction(
        period=0.5,  
        actions=[spawn_tether_node]
    )

    delayed_spawn_uav_node = TimerAction(
        period=0.1, 
        actions=[spawn_uav_node]
    )

    delayed_attach_links = TimerAction(
        period=0.5,  
        actions=[attach_tether_node]
    )

    nodes = [
        # Initialize Gazebo
        gzserver,
        gzclient, 

        # Initialize UGV controllers
        controller_manager,
        forward_position_controller,
        forward_velocity_controller,
        joint_state_broadcaster,
        ugv_state_publisher,
        
        # Spawn models
        spawn_ugv_node,
        spawn_uav_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_ugv_node,
                on_exit=[delayed_spawn_tether_node],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_tether_node,
                on_exit=[delayed_attach_links],
            )
        ),
        # spawn_theatre,
    ]

    return LaunchDescription(nodes)

