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
##### python3 /home/upo/marsupial/src/marsupial_simulator_ros2/scripts/jinja_gen.py /home/upo/marsupial/src/marsupial_simulator_ros2/models/tether/tether_lineal.sdf.jinja /home/upo/marsupial/src/marsupial_simulator_ros2/models/tether    

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
    init_pos_z = PythonExpression([init_pos_z, ' + 0.0'])

    uav_pos_x = PythonExpression([init_pos_x, ' + 0.0'])
    uav_pos_y = PythonExpression([init_pos_y, ' + 0.0'])
    uav_pos_z = PythonExpression([init_pos_z, ' + 0.4'])

    uav_2_pos_x = PythonExpression([init_pos_x, ' + 4.0'])
    uav_2_pos_y = PythonExpression([init_pos_y, ' + 0.0'])
    uav_2_pos_z = PythonExpression([init_pos_z, ' + 0.4'])

    tether_pos_x = PythonExpression([init_pos_x, ' - 0.0'])
    tether_pos_y = PythonExpression([init_pos_y, ' - 0.0'])
    tether_pos_z = PythonExpression([init_pos_z, ' + 0.1'])

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
    
    spawn_uav_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'sjtu_drone', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'sjtu_drone/sjtu_drone.sdf'), 
                       '-x', uav_pos_x, '-y', uav_pos_y, '-z', uav_pos_z],
            output='screen',
        )  
    spawn_uav_node_2 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'sjtu_drone_2', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'sjtu_drone/sjtu_drone.sdf'), 
                       '-x', uav_2_pos_x, '-y', uav_2_pos_y, '-z', uav_2_pos_z],
            output='screen',
        )  
    
    spawn_tether_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'tether', '-file', os.path.join(get_package_share_directory('marsupial_simulator_ros2'), 'models', 'tether/tether_lineal.sdf'), 
                       '-x', tether_pos_x, '-y', tether_pos_y, '-z', tether_pos_z],
            output='screen',
        ) 
    
    attach_tether_node = ExecuteProcess(
        cmd=['ros2', 'run', 'marsupial_simulator_ros2', 'attach_tether_lineal.py'],
        output='screen'
    )

    delayed_spawn_uav_node = TimerAction(
        period=1.0, 
        actions=[spawn_uav_node]
    )

    delayed_spawn_uav_node_2 = TimerAction(
        period=1.0, 
        actions=[spawn_uav_node_2]
    )

    delayed_attach_links = TimerAction(
        period=3.0,  
        actions=[attach_tether_node]
    )

    nodes = [
        # Initialize Gazebo
        gzserver,
        gzclient, 
        
        # Spawn models
        spawn_tether_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_tether_node,
                on_exit=[delayed_spawn_uav_node, delayed_spawn_uav_node_2],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_uav_node,
                on_exit=[delayed_attach_links],
            )
        ),
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=attach_tether_node,
        #         on_exit=[delayed_spawn_theatre],
        #     )
        # ),
    ]

    return LaunchDescription(nodes)

