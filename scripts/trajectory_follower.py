#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Float64MultiArray
import math
import sys
import os
from ament_index_python.packages import get_package_share_directory

class TrajectoryPublisher(Node):
    def __init__(self, mission_name):
        super().__init__('trajectory_publisher')
        self.ugv_poses = []
        self.uav_poses = []
        self.tether_lengths = []
        self.ugv_target_index = 0
        self.uav_target_index = 0
        self.tether_target_index = 0
        self.current_tether_length = 0.0
        self.current_ugv_pose = Pose()
        self.current_uav_pose = Pose()

        package_share_directory = get_package_share_directory('marsupial_simulator_ros2')
        mission_file_path = os.path.join(package_share_directory, 'optimized_path', f'{mission_name}.yaml')

        self.load_trajectories(mission_file_path)

        self.ugv_subscriber = self.create_subscription(Pose, '/ugv_gt_pose', self.ugv_pose_callback, 10)
        self.uav_subscriber = self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.uav_pose_callback, 10)
        self.tether_length_subscriber = self.create_subscription(Float64MultiArray, '/cable_length', self.tether_length_callback, 10)

        self.ugv_publisher = self.create_publisher(Pose, '/target_position_ugv', 10)
        self.uav_publisher = self.create_publisher(Pose, '/target_position_uav', 10)
        self.tether_publisher = self.create_publisher(Float64, '/target_length_tether', 10)

        self.timer = self.create_timer(1.0, self.publish_target_positions)

        self.publish_target_positions()

    def load_trajectories(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

        self.ugv_poses = [data['marsupial_ugv'][f'poses{i}']['pose'] for i in range(data['marsupial_ugv']['size'])]
        self.uav_poses = [data['marsupial_uav'][f'poses{i}']['pose'] for i in range(data['marsupial_uav']['size'])]
        self.tether_lengths = [data['tether'][f'length{i}']['length'] for i in range(data['tether']['size'])]
        self.get_logger().info(f'Loaded {len(self.ugv_poses)} UGV poses, {len(self.uav_poses)} UAV poses, and {len(self.tether_lengths)} tether lengths.')

    def ugv_pose_callback(self, msg):
        self.current_ugv_pose = msg
        self.check_and_update_targets()

    def uav_pose_callback(self, msg):
        self.current_uav_pose = msg
        self.check_and_update_targets()

    def tether_length_callback(self, msg):
        self.current_tether_length = msg.data[0]  
        self.check_and_update_targets()

    def check_and_update_targets(self):
        if (self.is_near_target_ugv(self.current_ugv_pose, self.ugv_poses[self.ugv_target_index]) and
            self.is_near_target_uav(self.current_uav_pose, self.uav_poses[self.uav_target_index]) and
            self.is_near_target_tether(self.current_tether_length, self.tether_lengths[self.tether_target_index])):

            self.get_logger().info(f'All conditions met, updating indices: UGV: {self.ugv_target_index}, UAV: {self.uav_target_index}, Tether: {self.tether_target_index}')
            self.ugv_target_index += 1
            self.uav_target_index += 1
            self.tether_target_index += 1

            if (self.ugv_target_index >= len(self.ugv_poses) or
                self.uav_target_index >= len(self.uav_poses) or
                self.tether_target_index >= len(self.tether_lengths)):
                
                self.get_logger().info("\033[92mTrayectoria finalizada\033[0m")
                self.timer.cancel()
                self.get_logger().info("Shutting down node.")
                rclpy.shutdown()
                return
            
            if self.ugv_target_index < len(self.ugv_poses):
                self.publish_target_position('ugv')
            if self.uav_target_index < len(self.uav_poses):
                self.publish_target_position('uav')
            if self.tether_target_index < len(self.tether_lengths):
                self.publish_target_position('tether')

    def is_near_target_uav(self, current_pose, target_pose):
        distance_threshold = 0.5        
        current_position = current_pose.position
        target_position = target_pose['position']
        distance = math.sqrt(
            (current_position.x - target_position['x'])**2 +
            (current_position.y - target_position['y'])**2 +
            (current_position.z - target_position['z'])**2
        )
        return distance < distance_threshold

    def is_near_target_ugv(self, current_pose, target_pose):
        distance_threshold = 0.3        
        current_position = current_pose.position
        target_position = target_pose['position']
        distance = math.sqrt(
            (current_position.x - target_position['x'])**2 +
            (current_position.y - target_position['y'])**2
        )
        return distance < distance_threshold

    def is_near_target_tether(self, current_length, target_length):
        tether_length_error_threshold = 0.3         
        length_error = abs(target_length - current_length)
        # self.get_logger().info(f'Tether length error: {length_error}, threshold: {tether_length_error_threshold}')
        return True # length_error < tether_length_error_threshold

    def publish_target_positions(self):
        self.publish_target_position('ugv')
        self.publish_target_position('uav')
        self.publish_target_position('tether')

    def publish_target_position(self, vehicle_type):
        if vehicle_type == 'ugv' and self.ugv_target_index < len(self.ugv_poses):
            pose_data = self.ugv_poses[self.ugv_target_index]
            pose_msg = Pose()
            pose_msg.position.x = float(pose_data['position']['x'])
            pose_msg.position.y = float(pose_data['position']['y'])
            pose_msg.position.z = float(pose_data['position']['z'])
            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            pose_msg.orientation.w = 1.0
            self.ugv_publisher.publish(pose_msg)
            # self.get_logger().info(f'Published UGV target position: {pose_msg.position.x}, {pose_msg.position.y}, {pose_msg.position.z}')

        elif vehicle_type == 'uav' and self.uav_target_index < len(self.uav_poses):
            pose_data = self.uav_poses[self.uav_target_index]
            pose_msg = Pose()
            pose_msg.position.x = float(pose_data['position']['x'])
            pose_msg.position.y = float(pose_data['position']['y'])
            pose_msg.position.z = float(pose_data['position']['z'])
            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            pose_msg.orientation.w = 1.0
            self.uav_publisher.publish(pose_msg)
            # self.get_logger().info(f'Published UAV target position: {pose_msg.position.x}, {pose_msg.position.y}, {pose_msg.position.z}')

        elif vehicle_type == 'tether' and self.tether_target_index < len(self.tether_lengths):
            tether_length = self.tether_lengths[self.tether_target_index]
            tether_msg = Float64()
            tether_msg.data = float(tether_length)
            self.tether_publisher.publish(tether_msg)
            # self.get_logger().info(f'Published tether target length: {tether_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    mission_name = sys.argv[1] if len(sys.argv) > 1 else 'test1'
    node = TrajectoryPublisher(mission_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
