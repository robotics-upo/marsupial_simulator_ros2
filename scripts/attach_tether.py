#!/usr/bin/env python3

import sys
import rclpy
from gazebo_ros_link_attacher.srv import Attach
from gazebo_msgs.srv import SpawnEntity
from copy import deepcopy
from transforms3d.euler import euler2quat


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('demo_attach_links')

    attach_srv = node.create_client(Attach, '/attach')
    while not attach_srv.wait_for_service(timeout_sec=1.0):
      node.get_logger().info(f'Waiting for service {attach_srv.srv_name}...')

    node.get_logger().info("Connected to service!")

    # From the shell:
    """
ros2 service call /attach 'gazebo_ros_link_attacher/srv/Attach' '{model_name_1: 'cube1',
link_name_1: 'link',
model_name_2: 'cube2',
link_name_2: 'link'}'
    """

    # Link drone to tether
    node.get_logger().info("Attaching sjtu_drone and tether")
    amsg = Attach.Request()
    amsg.model_name_1 = "sjtu_drone"
    amsg.link_name_1 = "base_link"
    amsg.model_name_2 = "tether"
    amsg.link_name_2 = "link_final"

    resp = attach_srv.call_async(amsg)
    rclpy.spin_until_future_complete(node, resp)

    node.get_logger().info("Published into linking service!")

    # Link winch to tether
    node.get_logger().info("Attaching winch and tether")
    amsg = Attach.Request()
    amsg.model_name_1 = "rs_robot"
    amsg.link_name_1 = "box_central"
    amsg.model_name_2 = "tether"
    amsg.link_name_2 = "link_0"

    resp = attach_srv.call_async(amsg)
    rclpy.spin_until_future_complete(node, resp)

    node.get_logger().info("Published into linking service!")


    