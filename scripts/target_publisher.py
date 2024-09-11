#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from marsupial_simulator_ros2.msg import TargetPoseUGV, TargetPoseUAV

class TargetPublisher(Node):

    def __init__(self):
        super().__init__('target_publisher')
        self.ugv_publisher_ = self.create_publisher(TargetPoseUGV, '/ugv_target_pose', 10)
        self.uav_publisher_ = self.create_publisher(TargetPoseUAV, '/uav_target_pose', 10)
        self.publish_target_poses()

    def publish_target_poses(self):
        ugv_target_pose = TargetPoseUGV()
        ugv_target_pose.x = 5.0
        ugv_target_pose.y = 3.0
        self.ugv_publisher_.publish(ugv_target_pose)
        self.get_logger().info('UGV target pose published: x={}, y={}'.format(ugv_target_pose.x, ugv_target_pose.y))

        uav_target_pose = TargetPoseUAV()
        uav_target_pose.x = 5.0
        uav_target_pose.y = 3.0
        uav_target_pose.z = 2.0  
        self.uav_publisher_.publish(uav_target_pose)
        self.get_logger().info('UAV target pose published: x={}, y={}, z={}'.format(uav_target_pose.x, uav_target_pose.y, uav_target_pose.z))

def main(args=None):
    rclpy.init(args=args)
    target_publisher = TargetPublisher()
    rclpy.spin(target_publisher)
    target_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
